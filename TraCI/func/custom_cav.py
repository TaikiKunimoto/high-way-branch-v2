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
LANE_CHANGE_MARGIN = 200.0  # [m] 渋滞発生地点の何メートル手前から車線変更を許可するか

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

        self.pos_x = 0
        self.pos_y = 0
        self.angle = None
        self.speed = 0
        self.accel = 0
        self.leader = None
        self.length = 5.0
        self.width = 1.8

        self.simTime = 0

        self.WithAgreementPhase = withAgree
        self.isWaitAgree = False

        self.collision_counter = 0
        self.caution_counter = 0
        self.emergency_brake_counter = 0

        self.departure_time = None
        self.arrival_time = None

        self.speed_history = []

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

        # 車線変更が可能なポイントを通過したら車線変更を可能にする
        if self.hasPassedLaneChangePoint(congestion_point):
            self.laneChangeStatus = "available"

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
        self._getFollowingVehicles(self.road, self.lane)
        print(
            f"{self.id} : right: {self.blocking_right_follower}, left: {self.blocking_left_follower}"
        )

    # 隣接車線の後続車両を取得
    def _getFollowingVehicles(self, road, lane):
        if road is None or lane is None or road != "MainLane1":
            self.blocking_left_follower = None
            self.blocking_right_follower = None
            return

        left_mode = 0b100  # left(0) + follower(0) + blocking only(1)
        right_mode = 0b101  # right(1) + follower(0) + blocking only(1)

        if lane == 0:
            left_follower = traci.vehicle.getNeighbors(self.id, left_mode)
            self.blocking_left_follower = left_follower if left_follower else None
            self.blocking_right_follower = None
        elif lane == 1:
            left_follower = traci.vehicle.getNeighbors(self.id, left_mode)
            right_follower = traci.vehicle.getNeighbors(self.id, right_mode)
            self.blocking_left_follower = left_follower if left_follower else None
            self.blocking_right_follower = right_follower if right_follower else None
        elif lane == 2:
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

        # 停止制動距離を計算
        stop_distance = self.speed**2 / (2 * abs(maxDecel))

        # 自車両が前方車両より速い場合
        if speed_diff > 0:
            min_duration = self._calculateSafeDecelDuration(speed_diff)
            ttc = self._calculateTTC(self.distance, speed_diff)
            if ttc >= min_duration:
                return
            else:
                # 減速
                duration = min(ttc, min_duration)
                traci.vehicle.slowDown(self.id, self.leader_speed, duration)
            # else:
            #     print("emergency brake")
            #     # 前方車両に近づきすぎている場合は急ブレーキ
            #     self.emergencyBreak(distance, speed_diff, leader_speed)
            #     return

    # 最大減速で速度差を0にするために必要な時間を計算
    def _calculateSafeDecelDuration(self, speed_diff):
        if speed_diff <= 0:
            return 0
        return speed_diff / abs(maxDecel)

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
    def emergencyBreak(self, ttc, targetSpeed):
        # stats.increment_emergency_brake()
        traci.vehicle.slowDown(self.id, targetSpeed, ttc)
