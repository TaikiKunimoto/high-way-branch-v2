import math
import os
import sys
from typing import Any, Dict, List, Optional, Tuple

from pydantic import BaseModel
from status.status import CarAction, CarStatus, LaneChangeStatus
from utils.traci_wrapper import (
    get_lane_last_step_veh_ids,
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

if "SUMO_HOME" in os.environ:
    tools = os.path.join(os.environ["SUMO_HOME"], "tools")
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")
import traci

# 定数
MAX_SPEED = 27  # [m/s]
MAX_ACCEL = 10.0  # [m/ss]
MAX_DECEL = -5.0  # [m/ss]
MIN_GAP = 2.8  # [m]
REACTION_TIME = 0.75  # [s]
FRICTION_COEFFICIENT = 0.7  # 摩擦係数
LANE_WIDTH = 3.2  # [m]
LANE_CHANGE_MARGIN_DEFAULT = 400.0  # [m] 通常時に分岐地点の何メートル手前から車線変更を許可するか
LANE_CHANGE_MARGIN_CONGESTED = 70.0  # [m] Lane2が混雑している際に渋滞最後尾の何m手前から車線変更を許可するか
SPEED_IMPROVEMENT_THRESHOLD = 40.0  # 車線変更による速度改善の閾値 [%]
MAINLANE_LENGTH = 2500  # [m]
TIME_STEP = 0.1  # [s]

# グローバルな車両管理辞書
vehicle_instances: Dict[str, "CustomCAV"] = {}


class CustomCAVParams(BaseModel):
    id: str
    type_id: Optional[str] = None
    route: Optional[str] = None
    road: Optional[str] = None
    lane_id: Optional[str] = None
    lane: Optional[int] = None
    lane_change_status: LaneChangeStatus = LaneChangeStatus.SPEED_IMPROVEMENT_ONLY
    status: CarStatus = CarStatus.NORMAL
    action: CarAction = CarAction.STAY
    priority: int = 0
    last_lane_change_time: Optional[float] = None
    receiving_cooperative_from_id: Optional[str] = None
    providing_cooperative_to_id: Optional[str] = None
    leader_distance: Optional[float] = None
    leader_speed: Optional[float] = None
    current_lane_leaders: Optional[List[Tuple[str, float]]] = None
    current_lane_followers: Optional[List[Tuple[str, float]]] = None
    left_followers: Optional[List[Tuple[str, float]]] = None
    right_followers: Optional[List[Tuple[str, float]]] = None
    left_leaders: Optional[List[Tuple[str, float]]] = None
    right_leaders: Optional[List[Tuple[str, float]]] = None
    lane_pos: Optional[float] = None
    pos_x: Optional[float] = None
    pos_y: Optional[float] = None
    speed: float = 0.0
    leader: Optional[Tuple[str, float]] = None
    length: float = 5.0
    width: float = 1.8
    reaction_distance: float = 0.0
    braking_distance: float = 0.0
    safety_gap: float = MIN_GAP
    do_not_speed_up: bool = False
    required_distance_from_leader: Optional[float] = None
    current_distance_from_leader: Optional[float] = None
    required_distance_from_follower: Optional[float] = None
    current_distance_from_follower: Optional[float] = None
    lane_change_leader_speed: Optional[float] = None
    sim_time: float = 0.0
    is_wait_agree: bool = False
    collision_counter: int = 0
    caution_counter: int = 0
    emergency_brake_counter: int = 0
    departure_time: Optional[float] = None
    arrival_time: Optional[float] = None
    speed_history: List[float] = []


class CustomCAV:
    def __init__(self, veh_id: int):
        self.params = CustomCAVParams(
            id=str(veh_id),
            type_id=get_veh_type(str(veh_id)),
            route=get_veh_route_id(str(veh_id)),
            lane_id=get_veh_lane_id(str(veh_id)),
        )
        vehicle_instances[self.params.id] = self

        # SUMOによる車線変更を無効化し、traciで速度管理
        traci.vehicle.setLaneChangeMode(vehID=self.params.id, laneChangeMode=0)
        traci.vehicle.setSpeedMode(vehID=self.params.id, speedMode=0)
        traci.vehicle.setMinGap(self.params.id, MIN_GAP)
        traci.vehicle.setTau(self.params.id, 1.0)

        # 車線変更ルールテーブルの初期化
        self.lane_change_rules: Dict[Tuple[int, str], Dict[str, Any]] = {
            # Lane 2 のルール
            (2, "speed"): {
                "action": CarAction.CHANGE_RIGHT,
                "priority": 0,
                "conditions": [
                    lambda: self.params.lane_change_status == LaneChangeStatus.SPEED_IMPROVEMENT_ONLY,
                    lambda: self._is_predicted_speed_increase("right"),
                ],
            },
            (2, "r_exit"): {"action": CarAction.STAY, "priority": 1, "conditions": []},
            (2, "r_pass"): {
                "action": CarAction.CHANGE_RIGHT,
                "priority": 3,
                "conditions": [
                    lambda: (
                        self.params.lane_change_status
                        in (
                            LaneChangeStatus.SPEED_IMPROVEMENT_ONLY,
                            LaneChangeStatus.ALL_ALLOWED,
                        )
                    ),
                    lambda: self._is_predicted_speed_increase("right"),
                ],
            },
            # Lane 1 のルール
            (1, "speed_left"): {
                "action": CarAction.CHANGE_LEFT,
                "priority": 0,
                "conditions": [
                    lambda: self.params.lane_change_status == LaneChangeStatus.SPEED_IMPROVEMENT_ONLY,
                    lambda: self._is_predicted_speed_increase("left"),
                ],
            },
            (1, "speed_right"): {
                "action": CarAction.CHANGE_RIGHT,
                "priority": 0,
                "conditions": [
                    lambda: self.params.lane_change_status == LaneChangeStatus.SPEED_IMPROVEMENT_ONLY,
                    lambda: self._is_predicted_speed_increase("right"),
                ],
            },
            (1, "r_exit"): {
                "action": CarAction.CHANGE_LEFT,
                "priority": 5,
                "conditions": [lambda: self.params.lane_change_status == LaneChangeStatus.ALL_ALLOWED],
            },
            (1, "r_pass_left"): {
                "action": CarAction.CHANGE_LEFT,
                "priority": 2,
                "conditions": [
                    lambda: self.params.lane_change_status == LaneChangeStatus.ALL_ALLOWED,
                    lambda: self._is_predicted_speed_increase("left"),
                ],
            },
            (1, "r_pass_right"): {
                "action": CarAction.CHANGE_RIGHT,
                "priority": 4,
                "conditions": [
                    lambda: self.params.lane_change_status == LaneChangeStatus.ALL_ALLOWED,
                    lambda: self._is_predicted_speed_increase("right"),
                ],
            },
            # Lane 0 のルール
            (0, "speed"): {
                "action": CarAction.CHANGE_LEFT,
                "priority": 0,
                "conditions": [
                    lambda: self.params.lane_change_status == LaneChangeStatus.SPEED_IMPROVEMENT_ONLY,
                    lambda: self._is_predicted_speed_increase("left"),
                ],
            },
            (0, "r_exit"): {
                "action": CarAction.CHANGE_LEFT,
                "priority": 6,
                "conditions": [lambda: self.params.lane_change_status == LaneChangeStatus.ALL_ALLOWED],
            },
            (0, "r_pass"): {
                "action": CarAction.CHANGE_LEFT,
                "priority": 2,
                "conditions": [
                    lambda: (
                        self.params.lane_change_status
                        in (
                            LaneChangeStatus.SPEED_IMPROVEMENT_ONLY,
                            LaneChangeStatus.ALL_ALLOWED,
                        )
                    ),
                    lambda: self._is_predicted_speed_increase("left"),
                ],
            },
        }

    def get_departure_time(self) -> None:
        """車輌の実際の出発時刻を取得"""
        self.params.departure_time = get_veh_departure(self.params.id)

    def get_arrival_time(self) -> None:
        """車輌の実際の到着時刻を取得"""
        self.params.arrival_time = get_sim_time()

    def update_status(self, congestion_point: Optional[float]) -> None:
        """自身のステータスを更新"""
        self.params.sim_time = get_sim_time()
        pos = get_veh_pos(self.params.id)
        self.params.pos_x, self.params.pos_y = pos[0], pos[1]
        self.params.lane_pos = get_veh_lane_position(self.params.id)
        self.params.speed = get_veh_speed(self.params.id)
        self.params.speed_history.append(self.params.speed)
        self.params.road = get_veh_road_id(self.params.id)
        self.params.lane = get_veh_lane_index(self.params.id)
        self.params.lane_id = get_veh_lane_id(self.params.id)
        self.params.leader = get_veh_leader(self.params.id, 0)
        self.params.leader_distance = self.params.leader[1] if self.params.leader is not None else None
        self.params.leader_speed = get_veh_speed(self.params.leader[0]) if self.params.leader is not None else None

        self._calculate_safety_gap()

        # 分流車両の車線変更が間に合わない場合、優先度を最大に設定
        # 分岐地点の50m手前で車線変更できていない場合は優先度を最大にする
        if self.params.road == "MainLane1" and self.params.lane_pos > MAINLANE_LENGTH - 50:
            if self.params.route == "r_exit" and self.params.lane != 2:
                self.params.priority = 7
                self.params.action = CarAction.CHANGE_LEFT
                self.params.status = CarStatus.LANE_CHANGING

        if self.params.road != "MainLane1" and self.params.status != CarStatus.NORMAL:
            self._reset_lane_change_state()

        # 車線変更が可能なポイントを通過したら車線変更を可能にする
        if self.params.lane_change_status == LaneChangeStatus.SPEED_IMPROVEMENT_ONLY:
            if self._has_passed_lane_change_point(congestion_point):
                self.params.lane_change_status = LaneChangeStatus.ALL_ALLOWED
                # 協調車線変更が可能になったタイミングで行動を初期化, 協調中であればそのステータスは維持
                self._reset_lane_change_state_keep_yielding()

        if self.params.lane_change_status == LaneChangeStatus.ALL_ALLOWED:
            if self.params.road != "MainLane1" and self.params.priority != 7:
                # 車線変更は禁止するが協調中のステータスは維持, 優先度が7の場合は車線変更可能
                self.params.lane_change_status = LaneChangeStatus.UNAVAILABLE
                self._reset_lane_change_state_keep_yielding()

        self._get_follower_and_leader()

        # 車線変更が可能かを判断するリストをリセット
        self.params.current_distance_from_follower = None
        self.params.required_distance_from_follower = None
        self.params.current_distance_from_leader = None
        self.params.required_distance_from_leader = None
        self.params.lane_change_leader_speed = None
        self.params.do_not_speed_up = False

        # 協調車両の整合性チェック（デバッグ用）
        if self.params.providing_cooperative_to_id is not None:
            if self.params.providing_cooperative_to_id in vehicle_instances:
                supporting_vehicle = vehicle_instances[
                    self.params.providing_cooperative_to_id
                ]  # 協調車両同士が同じレーンにいる場合は協調関係を解消
                if self.params.lane == supporting_vehicle.params.lane:
                    print(
                        f"Warning: {self.params.id} is providing cooperation to {self.params.providing_cooperative_to_id} but they are in the same lane"
                    )
                    supporting_vehicle._reset_lane_change_state()
                # 協調車両同士の情報が正しいか確認
                if supporting_vehicle.params.action == CarAction.CHANGE_LEFT:
                    if supporting_vehicle.params.lane != self.params.lane - 1:
                        print(
                            f"Warning: {self.params.id} is providing cooperation to {self.params.providing_cooperative_to_id} but the target lane is not correct"
                        )
                        supporting_vehicle._reset_lane_change_state()
                elif supporting_vehicle.params.action == CarAction.CHANGE_RIGHT:
                    if supporting_vehicle.params.lane != self.params.lane + 1:
                        print(
                            f"Warning: {self.params.id} is providing cooperation to {self.params.providing_cooperative_to_id} but the target lane is not correct"
                        )
                        supporting_vehicle._reset_lane_change_state()
                if self.params.id != supporting_vehicle.params.receiving_cooperative_from_id:
                    print(
                        f"Warning: {self.params.id} is providing cooperation to {self.params.providing_cooperative_to_id} but receiving cooperation from {supporting_vehicle.params.receiving_cooperative_from_id}"
                    )

    def decide_next_action_and_priority(self) -> None:
        """自身の行動（priority） を決定"""
        # 無効な道路上の場合は何もしない
        if self.params.road != "MainLane1":
            self.params.action = CarAction.STAY
            self.params.priority = 0

        # 車線変更中の場合は行動を継続
        if self.params.lane_change_status == LaneChangeStatus.ALL_ALLOWED and (
            self.params.status in (CarStatus.LANE_CHANGING, CarStatus.YIELDING)
        ):
            return

        # 現在のレーンと経路に基づくルールを取得
        if self.params.lane != 1:
            base_key = (self.params.lane, self.params.route)
            speed_key = (self.params.lane, "speed")
        else:
            if self.params.lane_change_status == LaneChangeStatus.SPEED_IMPROVEMENT_ONLY:
                base_key = (self.params.lane, self.params.route)
                if self._is_predicted_speed_increase("left"):
                    speed_key = (self.params.lane, "speed_left")
                elif self._is_predicted_speed_increase("right"):
                    speed_key = (self.params.lane, "speed_right")
                else:
                    speed_key = None
            else:
                speed_key = None
                if self.params.route == "r_exit":
                    base_key = (self.params.lane, self.params.route)
                else:
                    if self._is_predicted_speed_increase("left"):
                        # TODO
                        if self.params.route is not None:
                            base_key = (self.params.lane, self.params.route + "_left")
                    elif self._is_predicted_speed_increase("right"):
                        # TODO
                        if self.params.route is not None:
                            base_key = (self.params.lane, self.params.route + "_right")
                    else:
                        base_key = (self.params.lane, self.params.route)

        if self.params.lane_change_status == LaneChangeStatus.SPEED_IMPROVEMENT_ONLY:
            # TODO
            rule = self.lane_change_rules.get(speed_key)
        else:
            # TODO
            rule = self.lane_change_rules.get(base_key)

        if not rule:
            self.params.action = CarAction.STAY
            self.params.priority = 0
            return

        # 条件を満たすか確認
        if all(condition() for condition in rule["conditions"]):
            self.params.action = rule["action"]
            self.params.priority = rule["priority"]
        else:
            self.params.action = CarAction.STAY
            self.params.priority = 0

        self.params.status = CarStatus.LANE_CHANGING if self.params.action != CarAction.STAY else CarStatus.NORMAL

        # 連続して車線変更を行わないための制御
        # 車輌の優先度を付与した後にそれを容認するかどうかを判断する
        if self.params.last_lane_change_time is not None and self.params.priority != 7:
            if self.params.priority >= 5:
                if self.params.sim_time - self.params.last_lane_change_time < 1:
                    self.params.action = CarAction.STAY
                    self.params.priority = 0
                    self.params.status = CarStatus.NORMAL
                    return
            else:
                if self.params.sim_time - self.params.last_lane_change_time < 10:
                    self.params.action = CarAction.STAY
                    self.params.priority = 0
                    self.params.status = CarStatus.NORMAL
                    return

    def control_speed(self) -> None:
        """車両の速度調整"""
        if self.params.leader_distance is not None and self.params.leader_distance < MIN_GAP:
            if self.params.leader_speed is not None:
                self._emergency_break(self.params.leader_speed)
            return

        # 協調フェーズの場合は加速は行わない
        if self.params.status == CarStatus.YIELDING or self.params.status == CarStatus.LANE_CHANGING:
            self.params.do_not_speed_up = True
        else:
            self.params.do_not_speed_up = False

        # 現在のレーンと制限速度を取得
        # TODO
        if self.params.lane_id is None:
            return
        speed_limit = get_lane_max_speed(self.params.lane_id)

        # 前方車両がいない場合
        if self.params.leader is None:
            self._control_speed_by_speed_limit(speed_limit)
        else:
            # TODO
            if self.params.leader_speed is None:
                return
            speed_diff = self.params.speed - self.params.leader_speed
            min_duration = self._calculate_safe_decel_duration(speed_diff)
            # TODO
            if self.params.leader_distance is None:
                return
            ttc_with_safety_margin = self._calculate_ttc(self.params.leader_distance, speed_diff)
            # 前方車両との距離 > safety_gap の場合
            # TODO
            if self.params.leader_distance is None:
                return
            if self.params.leader_distance >= self.params.safety_gap:
                if speed_diff <= 0 or min_duration < ttc_with_safety_margin:
                    self._control_speed_by_speed_limit(speed_limit)
                else:
                    # 通常の減速
                    duration = min(ttc_with_safety_margin, min_duration)
                    traci.vehicle.slowDown(self.params.id, self.params.leader_speed, duration)
                    return
            # 前方車両との距離 < safety_gap の場合
            else:
                if self.params.do_not_speed_up:
                    return
                if speed_diff >= 0:
                    # 通常の減速
                    if self.params.leader_speed > 1:
                        target_speed = self.params.leader_speed - 1
                    else:
                        target_speed = 0
                    duration = min(ttc_with_safety_margin, min_duration)
                    traci.vehicle.slowDown(self.params.id, target_speed, duration)
                else:
                    return

    def execute_lane_change(self, lane_change_history: Dict[str, Dict[str, Any]]) -> None:
        """車線変更の実行"""
        if self.params.lane_change_status == LaneChangeStatus.UNAVAILABLE and self.params.priority != 7:
            return
        if self.params.action == CarAction.STAY:
            return

        # 速度向上を目的とする車線変更で協調は行わない
        # TODO Lane2からの速度向上を目的とする車線変更でどこまで協調させるか検討
        if self.params.priority >= 3:
            cooperation_mode = True
        else:
            cooperation_mode = False

        direction = "left" if self.params.action == CarAction.CHANGE_LEFT else "right"
        lane_change_amount = 1 if direction == "left" else -1
        # TODO
        if self.params.lane is None:
            return
        target_lane = self.params.lane + lane_change_amount

        if self._can_change_lane(direction, lane_change_history):
            if self.params.road != "MainLane1":
                self._reset_lane_change_state()
                return

            # 車線変更が可能な場合は実行
            traci.vehicle.changeLane(self.params.id, target_lane, 0)
            # 車線変更情報を辞書に記録
            lane_change_history[self.params.id] = {"lane": target_lane, "pos": self.params.lane_pos}
            self.params.last_lane_change_time = self.params.sim_time
            # 車線変更中のステータスをリセット
            self._reset_lane_change_state()
        else:
            if cooperation_mode:
                # 協調車輌は毎stepで探索するが、同一の場合にはリセットは行わない
                self._decide_yielding_vehicle()
                # 協調車輌と自身の速度を調整
                self._adjust_speed_for_cooperation()
            else:
                # 協調が許可されていない場合(速度向上車線変更)の場合は自身の速度のみ調整
                self._adjust_speed_for_cooperation()

    def _calculate_safety_gap(self) -> None:
        """適切な車間距離の計算"""
        speed_kmh = self.params.speed * 3.6
        # 空走距離
        self.params.reaction_distance = self.params.speed * REACTION_TIME
        # 制動距離
        self.params.braking_distance = (speed_kmh**2) / (254.016 * FRICTION_COEFFICIENT)
        # 安全距離
        self.params.safety_gap = self.params.reaction_distance + self.params.braking_distance + MIN_GAP

    def _get_follower_and_leader(self) -> None:
        """後続車両および先行車両の取得"""
        self._reset_follower_and_leader()
        if self.params.road is None or self.params.lane is None or self.params.road != "MainLane1":
            return

        own_position = self.params.lane_pos
        # TODO
        if own_position is None:
            return
        # レーン番号に基づいて確認すべき隣接レーンを決定
        check_lanes: List[Tuple[str, int]] = [("current", self.params.lane)]
        if self.params.lane == 0:
            check_lanes.append(("left", 1))  # レーン0の場合、左側のレーン1をチェック
        elif self.params.lane == 1:
            check_lanes.append(("right", 0))  # レーン1の場合、右側のレーン0をチェック
            check_lanes.append(("left", 2))  # レーン1の場合、左側のレーン2もチェック
        elif self.params.lane == 2:
            check_lanes.append(("right", 1))  # レーン2の場合、右側のレーン1をチェック

        for direction, lane_num in check_lanes:
            lane_id = f"{self.params.road}_{lane_num}"
            lane_vehicles = get_lane_last_step_veh_ids(lane_id)
            if not lane_vehicles:
                continue

            followers: List[Tuple[str, float]] = []
            leaders: List[Tuple[str, float]] = []

            for veh_id in lane_vehicles:
                if veh_id == self.params.id:
                    continue
                veh_pos = get_veh_lane_position(veh_id)
                distance = abs(own_position - veh_pos)
                if veh_pos < own_position:
                    followers.append((veh_id, distance))
                else:
                    leaders.append((veh_id, distance))

            followers.sort(key=lambda x: x[1])
            leaders.sort(key=lambda x: x[1])

            if direction == "left":
                self.params.left_followers = followers
                self.params.left_leaders = leaders
            elif direction == "right":
                self.params.right_followers = followers
                self.params.right_leaders = leaders
            elif direction == "current":
                self.params.current_lane_followers = followers
                self.params.current_lane_leaders = leaders

    def _reset_follower_and_leader(self) -> None:
        self.params.current_lane_followers = None
        self.params.current_lane_leaders = None
        self.params.left_followers = None
        self.params.left_leaders = None
        self.params.right_followers = None
        self.params.right_leaders = None

    def _has_passed_lane_change_point(self, congestion_point: Optional[float]) -> bool:
        """車線変更可能ポイントを通過しているか"""
        current_pos = self.params.lane_pos
        # TODO
        if current_pos is None:
            return False
        if (
            congestion_point is None
            or congestion_point > MAINLANE_LENGTH - LANE_CHANGE_MARGIN_DEFAULT + LANE_CHANGE_MARGIN_CONGESTED
        ):
            merge_start_pos = MAINLANE_LENGTH - LANE_CHANGE_MARGIN_DEFAULT
        else:
            merge_start_pos = congestion_point - LANE_CHANGE_MARGIN_CONGESTED
        return current_pos > merge_start_pos

    def _control_speed_by_speed_limit(self, speed_limit: float) -> None:
        """制限速度に合わせて速度調整"""
        # 減速
        if self.params.speed > speed_limit:
            safe_duration = self._calculate_safe_decel_duration(self.params.speed - speed_limit)
            traci.vehicle.slowDown(self.params.id, speed_limit, safe_duration)
        else:
            if not self.params.do_not_speed_up:
                safe_duration = self._calculate_safe_accel_duration(speed_limit - self.params.speed)
                traci.vehicle.slowDown(self.params.id, speed_limit, safe_duration)
                return

    def _calculate_safe_decel_duration(self, speed_diff: float) -> float:
        """最大減速で速度差を0にするために必要な時間を計算"""
        if speed_diff <= 0:
            return 0
        return speed_diff / abs(MAX_DECEL)

    def _calculate_safe_accel_duration(self, speed_diff: float) -> float:
        """最大加速で速度差を0にするために必要な時間を計算"""
        if speed_diff <= 0:
            return 0
        return speed_diff / abs(MAX_ACCEL)

    def _calculate_ttc(self, distance: float, speed_diff: float) -> float:
        """現在の速度差で進んだ際に衝突までにかかる時間(TTC)"""
        if speed_diff <= 0:
            return math.inf
        return distance / speed_diff

    def _emergency_break(self, target_speed: float) -> None:
        """衝突回避のための速度調整"""

        target_speed = min(target_speed, self.params.speed, 1)
        self.params.emergency_brake_counter += 1
        traci.vehicle.setSpeed(self.params.id, target_speed)

    def _request_cooperation(self) -> None:
        if self.params.receiving_cooperative_from_id in vehicle_instances:
            supporting_vehicle = vehicle_instances[self.params.receiving_cooperative_from_id]
            supporting_vehicle._reset_lane_change_state()
            supporting_vehicle.params.status = CarStatus.YIELDING
            supporting_vehicle.params.providing_cooperative_to_id = self.params.id

    def _adjust_speed_for_cooperation(self) -> None:
        """自身および協調車両の速度調整"""
        if self.params.leader_distance is not None and self.params.leader_distance < MIN_GAP:
            # TODO
            if self.params.leader_speed is not None:
                self._emergency_break(self.params.leader_speed)
            return

        # 自身の速度を車線変更に適した速度に調整
        # 目的車線の前方に車線変更を妨げる車両がいるなら,その車両の速度を参考にする
        if self.params.lane_change_leader_speed is not None:
            target_speed = self._calculate_supporting_speed(
                self.params.lane_change_leader_speed,
                self.params.current_distance_from_leader,
                self.params.required_distance_from_leader,
            )
            # 後方車輌が徐々に減速しているのでsafe_durationがstepごとに大きくなってしまい減速が遅くなるため明示的に指定してる
            traci.vehicle.slowDown(self.params.id, target_speed, 0.5)
        # 目的車線の前方に車線変更を妨げる車両がいないなら現在の車線のリーダーの速度を参考にする
        elif self.params.leader_speed is not None:
            traci.vehicle.slowDown(self.params.id, self.params.leader_speed, 0.5)
        # 前方に車両がいないなら制限速度に合わせる
        else:
            # 現在のレーンと制限速度を取得
            # TODO
            if self.params.lane_id is not None:
                speed_limit = get_lane_max_speed(self.params.lane_id)
            self._control_speed_by_speed_limit(speed_limit)

        # 協調車両がいる場合は相手の速度も調整
        if self.params.receiving_cooperative_from_id:
            supporting_vehicle = vehicle_instances[self.params.receiving_cooperative_from_id]
            own_position = self.params.lane_pos
            supporting_vehicle._adjust_supporting_speed(
                self.params.speed,
                own_position,
                self.params.current_distance_from_follower,
                self.params.required_distance_from_follower,
            )

    def _adjust_supporting_speed(
        self,
        requesting_speed: float,
        requesting_position: Optional[float],
        current_distance: Optional[float],
        required_distance: Optional[float],
    ) -> None:
        """車線変更を支援する側の速度の調整"""
        if self.params.leader_distance is not None and self.params.leader_distance < MIN_GAP:
            # TODO
            if self.params.leader_speed is not None:
                self._emergency_break(self.params.leader_speed)
            return

        # 安全な車間距離を確保しつつ速度を調整
        target_speed = self._calculate_supporting_speed(requesting_speed, current_distance, required_distance)
        safe_duration = self._calculate_safe_decel_duration(self.params.speed - target_speed)
        traci.vehicle.slowDown(self.params.id, target_speed, safe_duration)

    def _calculate_supporting_speed(
        self, requesting_speed: float, current_distance: Optional[float], required_distance: Optional[float]
    ) -> float:
        if current_distance is None or required_distance is None:
            return requesting_speed * 0.3

        position_diff = required_distance - current_distance

        # 車間距離が不足 → より大きく減速して車間を開ける
        deceleration_rate = min(position_diff / required_distance, 0.3)
        return requesting_speed * deceleration_rate

    def _decide_yielding_vehicle(self) -> None:
        """協調車両を決定"""
        candidates = None
        if self.params.action == CarAction.CHANGE_LEFT:
            candidates = self.params.left_followers
        elif self.params.action == CarAction.CHANGE_RIGHT:
            candidates = self.params.right_followers
        else:
            if self.params.receiving_cooperative_from_id is not None:
                self._reset_yielding_vehicle_state()
            return

        if not candidates:
            if self.params.receiving_cooperative_from_id is not None:
                self._reset_yielding_vehicle_state()
            return

        # 優先順位に基づいて候補を選定
        viable_candidates: List[Tuple[str, float]] = []
        new_receiving_cooperative_from_id: Optional[str] = None

        for veh_id, distance in candidates:
            if veh_id in vehicle_instances:
                veh = vehicle_instances[veh_id]
                if veh.params.priority < self.params.priority and (
                    veh.params.status == CarStatus.NORMAL or veh.params.status == CarStatus.LANE_CHANGING
                ):  # 自身より優先度が低い車輌に限定
                    viable_candidates.append((veh_id, distance))

        # 候補車両があれば、最も近い車両を選択
        if viable_candidates:
            new_receiving_cooperative_from_id = (
                viable_candidates[0][0]
                # 速度が0でない場合は最も近い車両を選択
                if self.params.speed != 0
                # 速度が0の場合は最も近い車両の次に近い車両を選択
                else (viable_candidates[1][0] if len(viable_candidates) > 1 else None)
            )

        if new_receiving_cooperative_from_id is None:
            if self.params.receiving_cooperative_from_id is not None:
                self._reset_yielding_vehicle_state()
            return

        if self.params.receiving_cooperative_from_id is not None:
            if new_receiving_cooperative_from_id != self.params.receiving_cooperative_from_id:
                # 新たに協調車両を決定するため過去の情報をリセット
                self._reset_yielding_vehicle_state()
                self.params.receiving_cooperative_from_id = new_receiving_cooperative_from_id
                self._request_cooperation()
            else:
                # 既に協調車両が決定されている場合はそのまま
                return
        else:
            self.params.receiving_cooperative_from_id = new_receiving_cooperative_from_id
            self._request_cooperation()

    def _reset_lane_change_state(self) -> None:
        self.params.status = CarStatus.NORMAL
        self.params.action = CarAction.STAY
        self.params.priority = 0
        self.params.required_distance_from_follower = None
        self.params.current_distance_from_follower = None
        self.params.required_distance_from_leader = None
        self.params.current_distance_from_leader = None
        self.params.lane_change_leader_speed = None
        self.params.do_not_speed_up = False
        self._reset_yielding_vehicle_state()

    def _reset_lane_change_state_keep_yielding(self) -> None:
        if self.params.status != CarStatus.YIELDING:
            self.params.status = CarStatus.NORMAL
        self.params.action = CarAction.STAY
        self.params.priority = 0
        self.params.required_distance_from_follower = None
        self.params.current_distance_from_follower = None
        self.params.required_distance_from_leader = None
        self.params.current_distance_from_leader = None
        self.params.lane_change_leader_speed = None
        self.params.do_not_speed_up = False
        self._reset_yielding_vehicle_state()

    def _reset_yielding_vehicle_state(self) -> None:
        if self.params.receiving_cooperative_from_id in vehicle_instances:
            supporting_vehicle = vehicle_instances[self.params.receiving_cooperative_from_id]
            supporting_vehicle.params.providing_cooperative_to_id = None
            supporting_vehicle.params.do_not_speed_up = False
            supporting_vehicle.params.status = (
                CarStatus.LANE_CHANGING if supporting_vehicle.params.action != CarAction.STAY else CarStatus.NORMAL
            )
            self.params.receiving_cooperative_from_id = None

    def _can_change_lane(self, direction: str, lane_change_history: Dict[str, Dict[str, Any]]) -> bool:
        """安全な車線変更が可能か判断"""
        # TODO
        if self.params.lane is None:
            return False
        target_lane = self.params.lane + (1 if direction == "left" else -1)
        check_range = self.params.length + MIN_GAP

        # 同一ステップでの車線変更を考慮する
        for _changed_veh_id, changed_info in lane_change_history.items():
            # 変更先レーンが同じかどうか確認
            if changed_info["lane"] == target_lane:
                # レーン上のポジションを比較（ここでは lane_pos での比較例）
                if abs(changed_info["pos"] - self.params.lane_pos) < check_range:
                    return False

        # 一つ挟んだ車線とのコリジョンを考慮
        if target_lane == 1 and self.params.road == "MainLane1":
            own_pos = self.params.lane_pos
            # TODO
            if own_pos is None:
                return False
            opposite_lane = 2 if self.params.lane == 0 else 0
            opposite_lane_vehicle_ids = get_lane_last_step_veh_ids(f"{self.params.road}_{opposite_lane}")
            for veh_id in opposite_lane_vehicle_ids:
                if veh_id in vehicle_instances:
                    opposite_vehicle = vehicle_instances[veh_id]
                    veh_pos = get_veh_lane_position(veh_id)
                    if (
                        abs(veh_pos - own_pos) < check_range
                        and opposite_vehicle.params.action != CarAction.STAY
                        and opposite_vehicle.params.priority >= self.params.priority
                    ):
                        return False

        followers = self.params.left_followers if direction == "left" else self.params.right_followers
        leaders = self.params.left_leaders if direction == "left" else self.params.right_leaders

        # 後続車両との安全性チェック
        if followers:
            follower_id, follower_distance = followers[0]
            if follower_id in vehicle_instances:
                follower = vehicle_instances[follower_id]
                speed_diff = follower.params.speed - self.params.speed
                required_distance = (
                    # 後続車両が遅い場合
                    # 最小限の車間距離のみ要求
                    self.params.length + MIN_GAP * 1.5
                    if speed_diff <= 0
                    # 必要な後続との距離は、車両長 + minGap + 後続の制動距離(+速度差)を考慮した値
                    # 速度差が大きい場合には制動距離をより考慮したい。速度差は 0 ~ 27のレンジなので、それを0 ~ 1に正規化して考慮する
                    else self.params.length + MIN_GAP * 1.5 + follower.params.safety_gap * (speed_diff / MAX_SPEED)
                )
                if follower_distance < required_distance:
                    self.params.current_distance_from_follower = follower_distance
                    self.params.required_distance_from_follower = required_distance

        # 先行車両との安全性チェック
        if leaders:
            leader_id, leader_distance = leaders[0]
            if leader_id in vehicle_instances:
                leader = vehicle_instances[leader_id]
                speed_diff = self.params.speed - leader.params.speed
                required_distance = (
                    # 自車両が遅い場合
                    # 最小限の車間距離のみ要求
                    self.params.length + MIN_GAP * 1.5
                    if speed_diff <= 0
                    # 車線変更時は通常のsafety_gapより短い距離を許容
                    else self.params.length + MIN_GAP * 1.5 + self.params.safety_gap * (speed_diff / MAX_SPEED)
                )
                if leader_distance < required_distance:
                    self.params.current_distance_from_leader = leader_distance
                    self.params.required_distance_from_leader = required_distance
                    self.params.lane_change_leader_speed = leader.params.speed

        if (
            self.params.required_distance_from_follower is not None
            or self.params.required_distance_from_leader is not None
        ):
            return False

        return True

    """ 車線変更を実行した際に速度が上昇するか """

    def _is_predicted_speed_increase(self, direction: str) -> bool:
        """車線変更後に速度向上が見込めるかを判断"""
        target_lane_leaders = (
            self.params.left_leaders
            if direction == "left"
            else (self.params.right_leaders if direction == "right" else None)
        )
        # TODO
        if not self.params.current_lane_leaders:
            return False

        if not target_lane_leaders:
            return True
        current_lane_leader_speeds = [get_veh_speed(leader[0]) for leader in self.params.current_lane_leaders]
        current_lane_avg_leader_speed = (
            sum(current_lane_leader_speeds) / len(current_lane_leader_speeds) if current_lane_leader_speeds else 0
        )
        # 自分自身の速度と走行中の車線の平均速度の大きい方を取得
        baseline_speed = max(self.params.speed, current_lane_avg_leader_speed)
        target_lane_leader_speeds = [get_veh_speed(leader[0]) for leader in target_lane_leaders]
        target_lane_avg_leader_speed = sum(target_lane_leader_speeds) / len(target_lane_leader_speeds)

        return target_lane_avg_leader_speed > baseline_speed * (1 + SPEED_IMPROVEMENT_THRESHOLD / 100)
