import copy
import math
import os
import sys
from re import S

import numpy as np

from simulationStatistics.simulation_statistics import SimulationStatistics

if "SUMO_HOME" in os.environ:
    tools = os.path.join(os.environ["SUMO_HOME"], "tools")
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")
import traci  # noqa

maxSpeed = 27  # [m/s]
maxAccel = 3.0  # [m/ss]
maxDecel = -5.0  # [m/ss]
minGap = 2.5  # [m]
maxPathLen = 10.0  # max path time [s]
LANE_WIDTH = 3.2  # [m]
reactionTime = 0.75  # [s]
frictionCoefficient = 0.7  # 摩擦係数

timeStep = 0.1  # [s]

mergeStartPos = 1100  # [m] 単純な手法の場合の車線変更開始地点(1500 - x)

# stats = SimulationStatistics()


class DefaultCAV:
    # constructor
    def __init__(self, vehID, withAgree=False):
        self.id = str(vehID)
        # 車両のデフォルトの車線変更モードを設定
        traci.vehicle.setLaneChangeMode(vehID=self.id, lcm=0b000000000000)
        # traci.vehicle.setLaneChangeMode(vehID=self.id, lcm=0b000000010100)
        # traci.vehicle.setLaneChangeMode(vehID=self.id, lcm=0b000000010000)

        # control vehicle speed by traci
        traci.vehicle.setSpeedMode(vehID=self.id, sm=0b000000)

        self.plannedPath = None
        self.desiredPaths = None
        self.receivedPaths = []
        self.alter_change_path = None
        self.alter_straight_path = None
        self.isPathValid = True
        self.pathID = 0
        self.typeID = traci.vehicle.getTypeID(self.id)
        self.route = traci.vehicle.getRouteID(self.id)
        self.road = None
        self.laneID = traci.vehicle.getLaneID(self.id)
        self.lane = None
        self.status = None  # free, follow, stop, lanechange, turn, yield
        self.distance = None  # 前方車両との距離
        self.leader_speed = None  # 前方車両の速度
        self.reaction_distance = 0  # 空走距離 [m]
        self.breaking_distance = 0  # 制動距離 [m]
        self.safety_gap = self.reaction_distance + self.breaking_distance + minGap

        self.pos_x = 0
        self.pos_y = 0
        self.angle = None
        self.speed = 0
        self.accel = 0
        self.leader = None
        self.leadPath = None
        self.length = 5.0
        self.width = 1.8

        self.simTime = 0
        self.myColor = [
            "blue",
            "navy",
            "steelblue",
            "deepskyblue",
            "cyan",
            "indigo",
            "royalblue",
        ][vehID % 7]
        self.yetJudgedList = []

        self.WithAgreementPhase = withAgree
        self.isWaitAgree = False
        self.waitPath = None
        self.waitAgreeID = []
        self.receiveAgreeID = []

        self.count_accept_utility = 0
        self.count_refuse_utility = 0
        self.count_refuse_physics = 0
        self.count_send_PT = 0
        self.count_send_DT = 0
        self.count_send_AT = 0

        self.collision_counter = 0
        self.caution_counter = 0
        self.emergency_brake_counter = 0

        self.departure_time = None
        self.arrival_time = None

        self.speed_history = []

    """ 車輌の実際の出発時刻を取得 """
    def get_departure_time(self):
        self.departure_time = traci.vehicle.getDeparture(self.id)

    """ 車輌の実際の到着時刻を取得 """ 
    def get_arrival_time(self):
        self.arrival_time = traci.simulation.getTime()

    """ 自身のステータスを更新 """
    def updateStatus(self):
        self.simTime = traci.simulation.getTime()
        self.speed_history.append(traci.vehicle.getSpeed(self.id))

        # 車線変更が可能なポイントを通過したら車線変更を可能にする
        if self.hasPassedLaneChangePoint():
            traci.vehicle.setLaneChangeMode(
                vehID=self.id, laneChangeMode=0b011000010101
            )

        # update own position
        pos = traci.vehicle.getPosition(self.id)
        self.pos_x = pos[0]
        self.pos_y = pos[1]

        self.angle = traci.vehicle.getAngle(self.id)
        self.speed = min(traci.vehicle.getSpeed(self.id), maxSpeed)
        self.accel = min(traci.vehicle.getAcceleration(self.id), maxAccel)
        self.road = traci.vehicle.getRoadID(self.id)
        self.lane = traci.vehicle.getLaneIndex(self.id)
        self.laneID = traci.vehicle.getLaneID(self.id)
        self.leader = traci.vehicle.getLeader(
            self.id, 0
        )  # 0 にすると制動距離より短い距離の先行車を取得
        self.distance = self.leader[1] if self.leader is not None else None
        self.leader_speed = (
            traci.vehicle.getSpeed(self.leader[0]) if self.leader is not None else None
        )

    """ 車線変更が可能なポイントを通過したかどうか """
    def hasPassedLaneChangePoint(self):
        if traci.vehicle.getLanePosition(self.id) > mergeStartPos:
            return True
        return False

    """ 車両の速度を制限速度に基づいて調整 """
    def controlSpeed(self):
        # 無効な道路上の場合は制御しない
        if self.road is None or self.road.startswith(":"):
            return

        # 現在のレーンと制限速度を取得
        current_lane = f"{self.road}_{self.lane}"
        speed_limit = traci.lane.getMaxSpeed(current_lane)

        # 制限速度を超えている場合は減速
        if self.speed > speed_limit:
            safe_duration = self._calculateSafeDuration(self.speed - speed_limit)
            traci.vehicle.slowDown(self.id, speed_limit, safe_duration)
        return

    """ 最大減速で速度差を0にするために必要な時間を計算 """
    def _calculateSafeDuration(self, speed_diff):
        if speed_diff <= 0:
            return 0
        return speed_diff / abs(maxDecel)
