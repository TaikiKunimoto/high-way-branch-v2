import math
import os
import sys
from typing import List, Optional, Tuple

from pydantic import BaseModel

from cav.constants import (
    FRICTION_COEFFICIENT,
    MAX_ACCEL,
    MAX_DECEL,
    MIN_GAP,
    REACTION_TIME,
)
from status.status import CarAction, CarStatus, LaneChangeStatus

if "SUMO_HOME" in os.environ:
    tools = os.path.join(os.environ["SUMO_HOME"], "tools")
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")
import traci


class BaseCAVParams(BaseModel):
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


class BaseCAV:
    params: BaseCAVParams

    def _calculate_safety_gap(self) -> None:
        """適切な車間距離の計算"""
        speed_kmh = self.params.speed * 3.6
        # 空走距離
        self.params.reaction_distance = self.params.speed * REACTION_TIME
        # 制動距離
        self.params.braking_distance = (speed_kmh**2) / (254.016 * FRICTION_COEFFICIENT)
        # 安全距離
        self.params.safety_gap = self.params.reaction_distance + self.params.braking_distance + MIN_GAP

    def _reset_follower_and_leader(self) -> None:
        self.params.current_lane_followers = None
        self.params.current_lane_leaders = None
        self.params.left_followers = None
        self.params.left_leaders = None
        self.params.right_followers = None
        self.params.right_leaders = None

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

    def _reset_yielding_vehicle_state(self) -> None:  # implemented per-subclass (uses module-global registry)
        raise NotImplementedError
