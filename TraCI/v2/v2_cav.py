"""v2 の車両クラス（自己完結）。

``cav.base_cav`` / ``cav.custom_cav`` からは継承・import しない。B1 では縦方向追従（control_speed）と
状態観測のみを持ち、車線変更（Layer2）・ペア形成（Layer1）は後続ブランチで追加する。
縦方向の追従ロジックは ``custom_cav`` の挙動を踏襲した自己完結実装。
"""

import math
import os
import sys

from pydantic import BaseModel

from status.status import CarStatus
from utils.traci_wrapper import (
    get_lane_max_speed,
    get_sim_time,
    get_veh_departure,
    get_veh_lane_id,
    get_veh_lane_index,
    get_veh_lane_position,
    get_veh_leader,
    get_veh_pos,
    get_veh_road_id,
    get_veh_route_id,
    get_veh_speed,
    get_veh_type,
)
from v2.constants import (
    DEGRADED_WAIT_SPEED,
    FRICTION_COEFFICIENT,
    MAX_ACCEL,
    MAX_DECEL,
    MIN_GAP,
    REACTION_TIME,
)

if "SUMO_HOME" in os.environ:
    tools = os.path.join(os.environ["SUMO_HOME"], "tools")
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")
import traci


class V2CAVParams(BaseModel):
    """v2 車両の状態。静的スカラー優先度（0–7）は持たない（優先度は EDF の鍵で都度決まる）。"""

    id: str
    type_id: str | None = None
    route: str | None = None
    road: str | None = None
    lane_id: str | None = None
    lane: int | None = None
    lane_pos: float | None = None
    pos_x: float | None = None
    pos_y: float | None = None
    speed: float = 0.0
    leader: tuple[str, float] | None = None
    leader_distance: float | None = None
    leader_speed: float | None = None
    length: float = 5.0
    safety_gap: float = MIN_GAP
    status: CarStatus = CarStatus.NORMAL
    do_not_speed_up: bool = False
    # Layer1 調停の役割（毎Tc フル再構築）。提供車↔要求車の対応を保持する。
    providing_to_id: str | None = None  # 自分が gap を提供している相手（要求車）
    receiving_from_id: str | None = None  # 自分に gap を提供してくれる相手（提供車）
    degraded: bool = False  # Θ_force 劣化モード（枠なし＆締切間際 → 安全減速で待機）
    # 必須LC要求の活性化（早め固定活性化）。活性化窓に初めて入った時刻を一度だけ記録する。
    activation_time: float | None = None
    activated: bool = False
    sim_time: float = 0.0
    departure_time: float | None = None
    arrival_time: float | None = None
    speed_history: list[float] = []
    emergency_brake_counter: int = 0


class V2CAV:
    """1車両のローカル制御（Layer2 の自車挙動）。Layer1 調停は ``rsu`` が外側で行う。"""

    params: V2CAVParams

    def __init__(self, veh_id: int) -> None:
        vid = str(veh_id)
        self.params = V2CAVParams(
            id=vid,
            type_id=get_veh_type(vid),
            route=get_veh_route_id(vid),
            lane_id=get_veh_lane_id(vid),
        )
        # SUMO の自律車線変更を無効化（車線変更は traci.changeLane のみ）。速度も traci で管理し、
        # 提供車の協調減速など制御介入を可能にする。
        traci.vehicle.setLaneChangeMode(self.params.id, 0)
        traci.vehicle.setSpeedMode(self.params.id, 0)
        traci.vehicle.setMinGap(self.params.id, MIN_GAP)
        traci.vehicle.setTau(self.params.id, 1.0)

    # --- 時刻記録 ---
    def record_departure_time(self) -> None:
        self.params.departure_time = get_veh_departure(self.params.id)

    def record_arrival_time(self) -> None:
        self.params.arrival_time = get_sim_time()

    # --- 状態観測 ---
    def update_observation(self) -> None:
        """毎step、自車の観測値を traci から取得して params を更新する。"""
        p = self.params
        p.sim_time = get_sim_time()
        pos = get_veh_pos(p.id)
        p.pos_x, p.pos_y = pos[0], pos[1]
        p.lane_pos = get_veh_lane_position(p.id)
        p.speed = get_veh_speed(p.id)
        p.speed_history.append(p.speed)
        p.road = get_veh_road_id(p.id)
        p.lane = get_veh_lane_index(p.id)
        p.lane_id = get_veh_lane_id(p.id)
        p.leader = get_veh_leader(p.id, 0)
        p.leader_distance = p.leader[1] if p.leader is not None else None
        p.leader_speed = get_veh_speed(p.leader[0]) if p.leader is not None else None
        self._calculate_safety_gap()

    # --- 縦方向制御（car-following）---
    def control_speed(self) -> None:
        """前方車両との車間に応じた速度制御。SUMO の安全制御は無効化済みのため自前で行う。"""
        p = self.params
        if p.leader_distance is not None and p.leader_distance < MIN_GAP:
            if p.leader_speed is not None:
                self._emergency_brake(p.leader_speed)
            return

        # Θ_force 劣化モード: 枠なし＆締切間際 → 無理なLCをせず安全減速で待機（gap が開くのを待つ）
        if p.degraded:
            target = min(p.speed, DEGRADED_WAIT_SPEED)
            if p.leader_speed is not None and p.leader_distance is not None and p.leader_distance < p.safety_gap:
                target = min(target, p.leader_speed)  # 近い前方車は尊重
            duration = self._safe_decel_duration(p.speed - target)
            traci.vehicle.slowDown(p.id, max(target, 0.0), duration)
            return

        # 協調・車線変更中は加速しない（B1 では常に NORMAL のため False）
        p.do_not_speed_up = p.status in (CarStatus.YIELDING, CarStatus.LANE_CHANGING)

        if p.lane_id is None:
            return
        speed_limit = get_lane_max_speed(p.lane_id)

        if p.leader is None or p.leader_speed is None or p.leader_distance is None:
            self._control_speed_by_speed_limit(speed_limit)
            return

        speed_diff = p.speed - p.leader_speed
        min_duration = self._safe_decel_duration(speed_diff)
        ttc = self._ttc(p.leader_distance, speed_diff)

        if p.leader_distance >= p.safety_gap:
            if speed_diff <= 0 or min_duration < ttc:
                self._control_speed_by_speed_limit(speed_limit)
            else:
                traci.vehicle.slowDown(p.id, p.leader_speed, min(ttc, min_duration))
        else:
            if p.do_not_speed_up:
                return
            if speed_diff >= 0:
                target_speed = p.leader_speed - 1 if p.leader_speed > 1 else 0.0
                traci.vehicle.slowDown(p.id, target_speed, min(ttc, min_duration))

    def _control_speed_by_speed_limit(self, speed_limit: float) -> None:
        """制限速度に合わせて加減速する。"""
        p = self.params
        if p.speed > speed_limit:
            traci.vehicle.slowDown(p.id, speed_limit, self._safe_decel_duration(p.speed - speed_limit))
        elif not p.do_not_speed_up:
            traci.vehicle.slowDown(p.id, speed_limit, self._safe_accel_duration(speed_limit - p.speed))

    def _emergency_brake(self, target_speed: float) -> None:
        """衝突回避の緊急減速。"""
        p = self.params
        p.emergency_brake_counter += 1
        traci.vehicle.setSpeed(p.id, min(target_speed, p.speed, 1.0))

    # --- 計算ヘルパ ---
    def _calculate_safety_gap(self) -> None:
        """追従の安全車間 = 空走距離 + 制動距離 + minGap。"""
        p = self.params
        speed_kmh = p.speed * 3.6
        reaction_distance = p.speed * REACTION_TIME
        braking_distance = (speed_kmh**2) / (254.016 * FRICTION_COEFFICIENT)
        p.safety_gap = reaction_distance + braking_distance + MIN_GAP

    @staticmethod
    def _safe_decel_duration(speed_diff: float) -> float:
        """最大減速で速度差を0にするのに要する時間。"""
        return speed_diff / abs(MAX_DECEL) if speed_diff > 0 else 0.0

    @staticmethod
    def _safe_accel_duration(speed_diff: float) -> float:
        """最大加速で速度差を0にするのに要する時間。"""
        return speed_diff / abs(MAX_ACCEL) if speed_diff > 0 else 0.0

    @staticmethod
    def _ttc(distance: float, speed_diff: float) -> float:
        """現在の速度差で進んだ際の衝突までの時間（TTC）。"""
        return distance / speed_diff if speed_diff > 0 else math.inf
