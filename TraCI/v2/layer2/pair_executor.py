"""Layer2: 実行。要求車は目標車線に十分なギャップがあれば（協調不要でも）瞬時LCし、無ければ提供車が協調減速で gap を開ける。

挿入が安全（前後二方向 OK）なら ``changeLane`` で瞬時に1段車線変更する。これは**提供車の有無に依らない**：
目標車線が空（合流先の専用車線・疎な車線など）でも、ギャップが安全なら自力で移れる（provider が居ないと一切動けない
旧挙動の修正）。安全でない要求のみ、Phase B で割り当てられた提供車を協調減速させ gap を広げる。本処理は control_speed
の後に呼び、協調減速の slowDown と changeLane が最後の指令になるようにする。
"""

import os
import sys

from utils.traci_wrapper import get_veh_neighbors, get_veh_speed
from v2.constants import HOLD_MARGIN, MAX_DECEL, MAX_SPEED, MIN_GAP
from v2.layer1.priority import EDF
from v2.layer1.rsu import Assignment
from v2.layer2.safety import VEH_LENGTH, Safety
from v2.lc_request import LCRequest
from v2.v2_cav import V2CAV

if "SUMO_HOME" in os.environ:
    tools = os.path.join(os.environ["SUMO_HOME"], "tools")
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")
import traci


class Layer2:
    """Layer2（実行層）：提供車の協調減速で gap を開け、安全なら要求車を瞬時LCする（状態を持たない静的ロジック）。"""

    @staticmethod
    def execute_pairs(
        assignments: list[Assignment],
        req_by_id: dict[str, LCRequest],
        by_id: dict[str, V2CAV],
    ) -> int:
        """各要求車を EDF 順に処理し、目標車線に十分なギャップがあれば瞬時LC、無ければ
        割り当てられた提供車を協調減速させて gap を開ける。実行したLC数を返す。

        要求は EDF 順（``req_by_id`` は鍵昇順の keyed から構築済み）に処理し、より緊急な要求車が先にスロットを
        確定する。前後二方向の安全（``_insertion_safe_live``＝getNeighbors でジャンクション跨ぎに実測）が満たされていれば
        提供車の有無に依らず changeLane する（空車線・疎な車線へも自力で移れる）。安全でない要求のみ、Phase B の割り当て提供車を協調減速させる。
        同一stepで同じ目標車線の重なる位置への二重挿入は ``committed`` で弾く（custom_cav の同一step同一レーンチェックに相当）。
        """
        provider_of = {a.requester_id: a.provider_id for a in assignments}
        lc_count = 0
        committed: dict[int, list[float]] = {}  # 今stepで確定した target_lane -> 縦位置のリスト
        for req in req_by_id.values():  # EDF 順（鍵昇順）
            requester = by_id.get(req.veh_id)
            if requester is None:
                continue
            target_lane = Safety.next_lane(req)
            if Layer2._insertion_safe_live(
                requester.id, requester.speed, target_lane < req.current_lane
            ) and Layer2._slot_free(committed, target_lane, req.current_pos):
                traci.vehicle.changeLane(requester.id, target_lane, 0)
                committed.setdefault(target_lane, []).append(req.current_pos)
                lc_count += 1
            else:
                provider_id = provider_of.get(req.veh_id)
                provider = by_id.get(provider_id) if provider_id is not None else None
                if provider is not None:
                    Layer2._provider_yield(provider, requester)
                if EDF.effective_distance(req) <= HOLD_MARGIN:
                    # 締切間近でなお挿入不可: D手前で滑らかに停止し行き止まり(lane-drop)の急停止を防ぐ（F5・最終手段）
                    Layer2._hold_before_deadline(requester, req)
                else:
                    # それ以前: 目標レーン先行の流速に自分を合わせ、速度差を縮めて挿入成功率を上げる（自己減速）
                    Layer2._requester_match_target_speed(requester, target_lane < req.current_lane)
        return lc_count

    @staticmethod
    def _slot_free(committed: dict[int, list[float]], target_lane: int, pos: float) -> bool:
        """今step、同じ目標車線で重なる位置への挿入が既に確定していないか（同時LC衝突を防ぐ）。"""
        check_range = VEH_LENGTH + MIN_GAP
        return all(abs(p - pos) >= check_range for p in committed.get(target_lane, []))

    @staticmethod
    def _insertion_safe_live(veh_id: str, ego_speed: float, going_right: bool) -> bool:
        """目標車線（going_right で左右決定）の実・後続/前走を getNeighbors で取得し、前後二方向の安全ギャップを満たすか判定する。

        snapshot（mainlane_edge 限定）と違い、フィーダーedge・内部ジャンクション車線から流入してくる車も
        getNeighbors がジャンクションを跨いで返すため、入口で流入車を見落とすブラインドスポット衝突を防ぐ。
        dist は minGap 込みの実ギャップ。接近側（後続が速い／自分が先行より速い）は g_req 相当の余裕を上乗せして要求する。
        """
        lat = 1 if going_right else 0  # bit1: 左=0 / 右=1
        base = MIN_GAP * 0.5
        for nid, dist in get_veh_neighbors(veh_id, lat):  # 後続（bit2=0）。後続が速いほど大きな車間が要る
            f_speed = get_veh_speed(nid)
            speed_diff = f_speed - ego_speed
            required = base + (Safety.g_req(f_speed) * (speed_diff / MAX_SPEED) if speed_diff > 0 else 0.0)
            if dist < required:
                return False
        for nid, dist in get_veh_neighbors(veh_id, lat | 2):  # 先行（bit2=1）。自分が速いほど大きな車間が要る
            speed_diff = ego_speed - get_veh_speed(nid)
            required = base + (Safety.g_req(ego_speed) * (speed_diff / MAX_SPEED) if speed_diff > 0 else 0.0)
            if dist < required:
                return False
        return True

    @staticmethod
    def _provider_yield(provider: V2CAV, requester: V2CAV) -> None:
        """提供車が協調減速して、要求車が入るための gap を開ける。"""
        p = provider
        r = requester
        if r.lane_pos is None or p.lane_pos is None:
            return
        current_gap = r.lane_pos - p.lane_pos  # 提供車は要求車より後方
        speed_diff = p.speed - r.speed
        required = VEH_LENGTH + MIN_GAP * 1.5
        if speed_diff > 0:
            required += Safety.g_req(p.speed) * (speed_diff / MAX_SPEED)
        target_speed = Layer2._supporting_speed(r.speed, current_gap, required)
        if p.speed > target_speed:
            duration = (p.speed - target_speed) / abs(MAX_DECEL)
            traci.vehicle.slowDown(p.id, max(target_speed, 0.0), duration)

    @staticmethod
    def _supporting_speed(requesting_speed: float, current_gap: float, required: float) -> float:
        """gap 不足分に応じて提供車の目標速度を下げる（要求車速度の 0〜30%）。"""
        if required <= 0:
            return requesting_speed
        position_diff = required - current_gap
        decel_rate = max(0.0, min(position_diff / required, 0.3))
        return requesting_speed * decel_rate

    @staticmethod
    def _hold_before_deadline(requester: V2CAV, req: LCRequest) -> None:
        """挿入できず提供車も無い要求車を、締切位置 D の手前で滑らかに減速・保持する（要求車自身の committed-wait, F5）。

        分岐直前まで巡航して SUMO トポロジー（teleport無効の lane-drop）で急停止するのを避ける挙動品質。EDF の鍵
        には載せない。D で停止する減速を MAX_DECEL 上限で slowDown する（dist ≤ HOLD_MARGIN の判定は呼び出し側）。
        leader が安全車間内に居る場合は control_speed の追従・緊急減速に委ねてスキップする：hold の弱い減速が
        緊急ブレーキを上書きすると hold 車列で後続が前車へ追突しうるため、先頭車（leader 遠い/無し）だけが hold する。
        """
        if requester.speed <= 0:
            return
        if requester.leader_distance is not None and requester.leader_distance < requester.safety_gap:
            return  # leader 近接: control_speed（追従・緊急ブレーキ）に委ねる
        remaining = req.deadline_pos - req.current_pos  # D までの残距離（要求は road==mainlane の同一フレームで生成）
        if remaining <= 0:
            return  # 既に D を越えていれば SUMO トポロジーに任せる
        needed_decel = (requester.speed**2) / (2 * remaining)  # D で停止するのに要する減速
        decel = min(needed_decel, abs(MAX_DECEL))  # 物理上限内で滑らかに（超過時は最大減速で best-effort）
        traci.vehicle.slowDown(requester.id, 0.0, requester.speed / decel)

    @staticmethod
    def _requester_match_target_speed(requester: V2CAV, going_right: bool) -> None:
        """要求車自身が目標レーンの先行車の速度まで減速し、速度差を縮めて挿入可能にする（合流のための自己減速）。

        最高速のまま低速の隣レーンへ突っ込めず一切減速しない問題への対処。目標レーン先行速度に合わせると
        ``_insertion_safe_live`` の必要ギャップ（速度差比例）が縮み、既存の物理ギャップへ滑り込める。提供車の後方
        ギャップ開けと相補的（提供車=後方／自己減速=自分が前方へ行き過ぎないよう速度を落とす）。own leader が
        安全車間内なら control_speed の追従・緊急減速に委ねてスキップする（緊急ブレーキ上書き防止）。
        """
        if requester.speed <= 0:
            return
        if requester.leader_distance is not None and requester.leader_distance < requester.safety_gap:
            return
        lat = (1 if going_right else 0) | 2  # 目標レーンの先行（bit2=1）
        front = get_veh_neighbors(requester.id, lat)
        if not front:
            return  # 前方車なし＝速度差ではなく別要因。自己減速しない
        lead_speed = min(get_veh_speed(nid) for nid, _ in front)
        if requester.speed <= lead_speed:
            return  # 既に同等以下なら減速不要
        duration = (requester.speed - lead_speed) / abs(MAX_DECEL)
        traci.vehicle.slowDown(requester.id, max(lead_speed, 0.0), duration)
