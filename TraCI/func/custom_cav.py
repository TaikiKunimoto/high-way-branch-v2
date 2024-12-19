import copy
import math
import os
import sys
from re import S

import numpy as np
from SimulationStatistics.simulation_statistics import SimulationStatistics

if "SUMO_HOME" in os.environ:
    tools = os.path.join(os.environ["SUMO_HOME"], "tools")
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")
import traci  # noqa

maxSpeed = 27  # [m/s]
maxAccel = 3.0  # [m/ss]
maxDecel = -5.0  # [m/ss]
LANE_WIDTH = 3.2  # [m]
LANE_CHANGE_MARGIN = 400.0  # [m] 渋滞発生地点の何メートル手前から車線変更を許可するか

timeStep = 0.1  # [s]

# stats = SimulationStatistics()


class CustomCAV:
    # constructor
    def __init__(self, vehID, alpha, withAgree=False):
        self.id = str(vehID)

        # sumoによる車線変更を無効化
        traci.vehicle.setLaneChangeMode(vehID=self.id, lcm=0)
        # control vehicle speed by traci
        traci.vehicle.setSpeedMode(vehID=self.id, sm=0)
        traci.vehicle.setMinGap(self.id, 2.5)  # default 2.5
        traci.vehicle.setTau(self.id, 1.0)  # default 1.0

        self.typeID = traci.vehicle.getTypeID(self.id)
        self.route = traci.vehicle.getRouteID(self.id)
        self.road = None
        self.laneID = traci.vehicle.getLaneID(self.id)
        self.lane = None
        self.laneChangeStatus = "unavailable"  # available, unavailable
        self.status = "straight"  # pending, straight
        self.priority = None  # 1(high), 2, 3, 4, 5(low)
        self.distance = None  # 前方車両との距離
        self.leader_speed = None  # 前方車両の速度
        self.blocking_left_follower = None  # 車線変更を妨げる左後続車両
        self.blocking_right_follower = None  # 車線変更を妨げる右後続車両
        self.left_leader = None  # 左前方車両
        self.right_leader = None  # 右前方車両

        self.pos_x = 0
        self.pos_y = 0
        self.angle = None
        self.speed = 0
        self.accel = 0
        self.leader = None
        self.length = 5.0
        self.width = 1.8
        self.minGap = 2.5

        self.simTime = 0

        self.WithAgreementPhase = withAgree
        self.isWaitAgree = False

        self.collision_counter = 0
        self.caution_counter = 0
        self.emergency_brake_counter = 0

        self.departure_time = None
        self.arrival_time = None

        self.speed_history = []

        # 車線変更ルールテーブルの初期化
        self.lane_change_rules = {
            # Lane 2のルール
            (2, "r_exit"): {"action": "stay", "conditions": []},
            (2, "r_pass"): {
                "action": "change_right",
                "conditions": [
                    lambda: self.laneChangeStatus == "available",
                    lambda: self._isPredictedSpeedIncrease("right"),
                    lambda: self._isLaneChangeSafe("right"),
                ],
            },
            # Lane 1のルール
            (1, "r_exit"): {
                "action": "change_left",
                "conditions": [
                    lambda: self.laneChangeStatus == "available",
                    lambda: self._isLaneChangeSafe("left"),
                ],
            },
            (1, "r_pass"): {
                "action": "change_right",
                "conditions": [
                    lambda: self.laneChangeStatus == "available",
                    lambda: self._isPredictedSpeedIncrease("right"),
                    lambda: self._isLaneChangeSafe("right"),
                ],
            },
            # Lane 0のルール
            (0, "r_exit"): {
                "action": "change_left",
                "conditions": [
                    lambda: self.laneChangeStatus == "available",
                    lambda: self._isLaneChangeSafe("left"),
                ],
            },
            (0, "r_pass"): {"action": "stay", "conditions": []},
        }

    # 車輌の実際の出発時刻を取得
    def get_departure_time(self):
        self.departure_time = traci.vehicle.getDeparture(self.id)

    # 車輌の実際の到着時刻を取得
    def get_arrival_time(self):
        self.arrival_time = traci.simulation.getTime()

    # 自身のステータスを更新
    def updateStatus(self, congestion_point):
        self.simTime = traci.simulation.getTime()
        self.speed_history.append(traci.vehicle.getSpeed(self.id))

        # update own position
        pos = traci.vehicle.getPosition(self.id)
        self.pos_x = pos[0]
        self.pos_y = pos[1]

        self.angle = traci.vehicle.getAngle(self.id)
        self.speed = traci.vehicle.getSpeed(self.id)
        self.accel = traci.vehicle.getAcceleration(self.id)
        self.road = traci.vehicle.getRoadID(self.id)
        self.lane = traci.vehicle.getLaneIndex(self.id)
        self.laneID = traci.vehicle.getLaneID(self.id)
        self.leader = traci.vehicle.getLeader(self.id, 0)
        self.distance = self.leader[1] if self.leader is not None else None
        self.leader_speed = (
            traci.vehicle.getSpeed(self.leader[0]) if self.leader is not None else None
        )

        # 車線変更が可能なポイントを通過したら車線変更を可能にする
        if self.hasPassedLaneChangePoint(congestion_point):
            self.laneChangeStatus = "available"
        if self.road != "MainLane1":
            self.laneChangeStatus = "unavailable"

        self._getFollowingVehicles()
        self._getLeadingVehicles()

    # 隣接車線の先行車輌を取得
    def _getLeadingVehicles(self):
        if self.road is None or self.lane is None:
            self.left_leader = None
            self.right_leader = None
            return

        left_mode = 0b010  # all(0) + leader(1) + left(0)
        right_mode = 0b011  # all(0) + leader(1) + right(1)

        if self.lane == 0:
            left_leader = traci.vehicle.getNeighbors(self.id, left_mode)
            self.left_leader = left_leader if left_leader else None
            self.right_leader = None
        elif self.lane == 1:
            left_leader = traci.vehicle.getNeighbors(self.id, left_mode)
            right_leader = traci.vehicle.getNeighbors(self.id, right_mode)
            self.left_leader = left_leader if left_leader else None
            self.right_leader = right_leader if right_leader else None
        elif self.lane == 2:
            right_leader = traci.vehicle.getNeighbors(self.id, right_mode)
            self.left_leader = None
            self.right_leader = right_leader if right_leader else None

    # 隣接車線の後続車両を取得
    def _getFollowingVehicles(self):
        if self.laneChangeStatus == "unavailable":
            self.blocking_left_follower = None
            self.blocking_right_follower = None
            return

        if self.road is None or self.lane is None or self.road != "MainLane1":
            self.blocking_left_follower = None
            self.blocking_right_follower = None
            return

        left_mode = 0b100  # blocking only(1) + follower(0) + left(0)
        right_mode = 0b101  # blocking only(1) + follower(0) + right(1)

        if self.lane == 0:
            left_follower = traci.vehicle.getNeighbors(self.id, left_mode)
            self.blocking_left_follower = left_follower if left_follower else None
            self.blocking_right_follower = None
        elif self.lane == 1:
            left_follower = traci.vehicle.getNeighbors(self.id, left_mode)
            right_follower = traci.vehicle.getNeighbors(self.id, right_mode)
            self.blocking_left_follower = left_follower if left_follower else None
            self.blocking_right_follower = right_follower if right_follower else None
        elif self.lane == 2:
            right_follower = traci.vehicle.getNeighbors(self.id, right_mode)
            self.blocking_left_follower = None
            self.blocking_right_follower = right_follower if right_follower else None

    # 車線変更が可能なポイントを通過したかどうか
    def hasPassedLaneChangePoint(self, congestion_point):
        lane_length = traci.lane.getLength("MainLane1_2")
        current_pos = traci.vehicle.getLanePosition(self.id)

        if congestion_point is None:
            if current_pos > lane_length - LANE_CHANGE_MARGIN:
                return True
            return False

        merge_start_pos = congestion_point - LANE_CHANGE_MARGIN

        if current_pos > merge_start_pos:
            return True
        return False

    # 車両の速度を調整
    def controlSpeed(self):
        # 無効な道路上の場合は制御しない
        if self.road is None or self.road.startswith(":"):
            return

        # 現在のレーンと制限速度を取得
        current_lane = f"{self.road}_{self.lane}"
        speed_limit = traci.lane.getMaxSpeed(current_lane)

        # 前方に車両がない場合の制御
        if self.leader is None:
            # 減速
            if self.speed > speed_limit:
                safe_duration = self._calculateSafeDecelDuration(
                    self.speed - speed_limit
                )
                traci.vehicle.slowDown(self.id, speed_limit, safe_duration)
            # 加速
            else:
                safe_duration = self._calculateSafeAccelDuration(
                    speed_limit - self.speed
                )
                traci.vehicle.slowDown(self.id, speed_limit, safe_duration)
            return

        # 前方車両がある場合の制御
        speed_diff = self.speed - self.leader_speed

        # 自車両が前方車両より速い場合
        if speed_diff > 0:
            min_duration = self._calculateSafeDecelDuration(speed_diff)
            ttc = self._calculateTTC(self.distance, speed_diff)
            print(f"min_duration: {min_duration}, ttc: {ttc}")

            if self.distance < 2.5:
                # 前方車両に近づきすぎている場合は急ブレーキ
                self._emergencyBreak(ttc, self.leader_speed)
                return

            if ttc >= min_duration:
                return
            else:
                # 減速
                duration = min(ttc, min_duration)
                traci.vehicle.slowDown(self.id, self.leader_speed, duration)

    # 最大減速で速度差を0にするために必要な時間を計算
    def _calculateSafeDecelDuration(self, speed_diff):
        if speed_diff <= 0:
            return 0
        return speed_diff / abs(maxDecel) * 2 # 2倍の余裕を持たせる

    def _calculateSafeAccelDuration(self, speed_diff):
        if speed_diff <= 0:
            return 0
        return speed_diff / abs(maxAccel)

    # 現在の速度差で進んだ際に衝突までにかかる時間(TTC)
    def _calculateTTC(self, distance, speed_diff):
        if speed_diff <= 0:
            return None
        return distance / speed_diff

    # 衝突回避のための速度調整
    def _emergencyBreak(self, ttc, targetSpeed):
        print("emergency brake")
        print(f"ttc: {ttc}, targetSpeed: {targetSpeed}")
        # stats.increment_emergency_brake()
        traci.vehicle.slowDown(self.id, targetSpeed, ttc)

    # 車線変更を実行
    def executeLaneChange(self):
        action = self.decideLaneChange()

        if self.speed == 0:
            tmp = self._isLaneChangeSafe("left")
            print(
                f"vehicle: {self.id}, lane: {self.lane}, route: {self.route}, action: {action}, laneChangeStatus: {self.laneChangeStatus}, isLaneChangeSafe: {tmp}"
            )

        if action == "change_left":
            traci.vehicle.changeLane(self.id, self.lane + 1, 0)
        elif action == "change_right":
            traci.vehicle.changeLane(self.id, self.lane - 1, 0)

    # 車線変更の判断を行う
    def decideLaneChange(self):
        # 無効な道路上の場合は何もしない
        if self.road is None or self.road.startswith(":"):
            return "stay"

        # 現在のレーンと経路に基づくルールを取得
        key = (self.lane, self.route)
        rule = self.lane_change_rules.get(key)

        if not rule:
            return "stay"

        # 全ての条件を満たすか確認
        if all(condition() for condition in rule["conditions"]):
            return rule["action"]
        return "stay"

    # 車線変更が安全かどうか
    def _isLaneChangeSafe(self, direction):
        blocking_follower = (
            self.blocking_left_follower
            if direction == "left"
            else self.blocking_right_follower
        )
        leader = self.left_leader if direction == "left" else self.right_leader

        # 後続車との衝突チェック
        collision_with_follower = blocking_follower is not None

        # 先行車との衝突チェック
        minimum_safe_distance = self.length + self.minGap
        collision_with_leader = leader and leader[0][1] < minimum_safe_distance

        # 両方の衝突チェックがFalseの場合のみ安全
        return not (collision_with_follower or collision_with_leader)

    # 車線変更を実行した際に速度が上昇するか
    def _isPredictedSpeedIncrease(self, direction):
        if direction == "left":
            left_leader_id = self.left_leader[0][0] if self.left_leader else None
            if left_leader_id is None:
                return False
            left_leader_speed = traci.vehicle.getSpeed(left_leader_id)
            return left_leader_speed > self.speed
        elif direction == "right":
            right_leader_id = self.right_leader[0][0] if self.right_leader else None
            if right_leader_id is None:
                return False
            right_leader_speed = traci.vehicle.getSpeed(right_leader_id)
            return right_leader_speed > self.speed
        return False
