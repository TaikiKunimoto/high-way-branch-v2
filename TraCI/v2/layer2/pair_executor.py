"""Layer2: ペア実行。提供車が協調減速で gap を開け、要求車は安全なら瞬時LCする。

挿入が安全（前後二方向 OK）なら ``changeLane`` で瞬時に1段車線変更。まだ安全でなければ提供車を協調減速させ
gap を広げる。本処理は control_speed の後に呼び、協調減速の slowDown と changeLane が最後の指令になるようにする。
"""

import os
import sys

from v2.constants import MAX_DECEL, MAX_SPEED, MIN_GAP
from v2.layer1.rsu import Assignment
from v2.layer2.safety import VEH_LENGTH, Safety
from v2.lc_request import LCRequest
from v2.snapshot import Snapshot
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
        snap: Snapshot,
        by_id: dict[str, V2CAV],
    ) -> int:
        """各ペアについて、安全なら要求車を瞬時LC、まだなら提供車を協調減速。実行したLC数を返す。

        assignments は鍵順（EDF順）なので、より緊急な要求車が先にスロットを確定する。同一stepで同じ目標車線の
        重なる位置への二重挿入は ``committed`` で弾く（custom_cav の同一step同一レーンチェックに相当）。
        """
        lc_count = 0
        committed: dict[int, list[float]] = {}  # 今stepで確定した target_lane -> 縦位置のリスト
        for a in assignments:
            req = req_by_id.get(a.requester_id)
            requester = by_id.get(a.requester_id)
            provider = by_id.get(a.provider_id)
            if req is None or requester is None or provider is None:
                continue
            target_lane = Safety.next_lane(req)
            if Safety.is_insertion_safe(req, requester.speed, snap) and Layer2._slot_free(
                committed, target_lane, req.current_pos
            ):
                traci.vehicle.changeLane(requester.id, target_lane, 0)
                committed.setdefault(target_lane, []).append(req.current_pos)
                lc_count += 1
            else:
                Layer2._provider_yield(provider, requester)
        return lc_count

    @staticmethod
    def _slot_free(committed: dict[int, list[float]], target_lane: int, pos: float) -> bool:
        """今step、同じ目標車線で重なる位置への挿入が既に確定していないか（同時LC衝突を防ぐ）。"""
        check_range = VEH_LENGTH + MIN_GAP
        return all(abs(p - pos) >= check_range for p in committed.get(target_lane, []))

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
