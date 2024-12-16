import copy
import math
import os
import sys
from re import S

import numpy as np

if "SUMO_HOME" in os.environ:
    tools = os.path.join(os.environ["SUMO_HOME"], "tools")
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")
import traci  # noqa
from PathPlanning.FrenetOptimalTrajectory.frenet_merge import (
    emergency_stop,
    generate_frenet_frame,
)

maxSpeed = 16.67  # [m/s]
maxAccel = 3.0  # [m/ss]
maxDecel = -5.0  # [m/ss]
sensorRange = 200  # [m]
commRange = 200  # [m]
maxPathLen = 10.0  # max path time [s]
LANE_WIDTH = 3.2  # [m]

timeStep = 0.1  # [s]
predTime = 10.0  # [s]
pathValidTime = 1.0  # [s]

D_0 = 5.0  # longitudinal safe margin [m]
tau = 1.5  # safe headway time [s]
PET_th = 3.0  # safe PET [s]
yield_th = 0.0  # threshold whether to yield

mergeStartPos = 150
all_follow_Range = False
follow_range = 200
collision_distance = 3.0
caution_distance = 6.0
consider_3s_later = False
divide_DT = False


class CAV:
    # constructor
    def __init__(self, vehID, alpha, withAgree=False):
        self.id = str(vehID)
        # 車両のデフォルトの車線変更モードを設定
        traci.vehicle.setLaneChangeMode(vehID=self.id, lcm=0b011001010101)
        # control vehicle speed by traci
        traci.vehicle.setSpeedMode(vehID=self.id, sm=0b100000)

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
        self.alpha = alpha
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
    def updateStatus(self, running_list):
        self.simTime = traci.simulation.getTime()
        self.speed_history.append(traci.vehicle.getSpeed(self.id))

        # update own position
        pos = traci.vehicle.getPosition(self.id)
        self.pos_x = pos[0]
        self.pos_y = pos[1]

        if self.plannedPath:
            for i in range(len(self.plannedPath.t)):
                if self.plannedPath.t[i] == self.simTime:
                    self.pos_x = self.plannedPath.x[i]
                    self.pos_y = self.plannedPath.y[i]

        self.angle = traci.vehicle.getAngle(self.id)
        self.speed = min(traci.vehicle.getSpeed(self.id), maxSpeed)
        self.accel = min(traci.vehicle.getAcceleration(self.id), maxAccel)
        self.road = traci.vehicle.getRoadID(self.id)
        self.lane = traci.vehicle.getLaneIndex(self.id)
        self.laneID = traci.vehicle.getLaneID(self.id)

        # leaderがシミュレーション範囲から出たらleaderを初期化
        if self.leader not in running_list:
            self.leader = None
            self.leadPath = None

        # 車線変更が完了したらlanechange状態を解除
        if self.status == "lanechange" and self.pos_y == self.plannedPath.y[-1]:
            self.status = None

        elif self.status == "yield":  # 譲った相手が車線変更をしたらyield状態を解除
            if self.lane == traci.vehicle.getLaneIndex(self.leader):
                self.status = None
                self.leader = None
                self.leadPath = None
                self.isPathValid = False

        # update own status
        ld = traci.vehicle.getLeader(self.id)
        if ld:
            if self.status == "yield":
                leadPath = [
                    path
                    for path in self.receivedPaths
                    if path.vehID == self.leader and path.isLaneChange == True
                ]
                if leadPath:
                    self.leadPath = leadPath[0]

                if self.leader == ld[0]:
                    self.status = "follow"

            elif (self.pos_x - traci.vehicle.getPosition(ld[0])[0]) ** 2 + (
                self.pos_y - traci.vehicle.getPosition(ld[0])[1]
            ) ** 2 <= sensorRange**2 and traci.vehicle.getRoadID(ld[0]):
                self.leader = ld[0]
                leadPath = [
                    path
                    for path in self.receivedPaths
                    if path.vehID == self.leader and path.type == "planned"
                ]
                if leadPath:
                    if (
                        self.leadPath
                        and self.leadPath.pathID != leadPath[0].pathID
                        and self.status not in ["yield", "turn"]
                    ):
                        # 直前の予定経路が変更された場合、自身の経路も合わせて再設定
                        self.isPathValid = False
                    self.leadPath = leadPath[0]

                # 前方車両が車線変更中の場合は、さらに一つ前の車両に追従する
                if (
                    self.leadPath
                    and self.plannedPath
                    and self.leadPath.isLaneChange
                    and (self.plannedPath.y[-1] != self.leadPath.y[-1])
                ):
                    ld = traci.vehicle.getLeader(self.leadPath.vehID)
                    self.leader = ld[0]

        elif ld is None:
            if self.status not in ["yield", "lanechange"]:
                self.leader = None
                self.status = "free"

        # set leading path
        if self.leader:
            leadPath = [
                path
                for path in self.receivedPaths
                if path.vehID == self.leader and path.type == "planned"
            ]
            if leadPath:
                leadPath = leadPath[0]
            else:
                self.leadPath = None
        else:
            self.leadPath = None

        self.count_accept_utility = 0
        self.count_refuse_utility = 0
        self.count_refuse_physics = 0
        self.count_send_PT = 0
        self.count_send_DT = 0
        self.count_send_AT = 0

    # 現在の道路の制限速度を確認し、必要に応じて速度を制限する
    def checkSpeedLimit(self):
        # 車両が有効なエッジ上にある場合のみ処理を行う
        if self.road is None or self.road.startswith(":"):
            return
            # 現在のレーンの制限速度を取得 (m/s)
        try:
            current_lane = f"{self.road}_{self.lane}"
            speed_limit = traci.lane.getMaxSpeed(current_lane)

            # 現在の速度が制限速度を超えている場合
            if self.speed > speed_limit:
                # target_speed = min(self.speed * 0.9, speed_limit) # 急ブレーキを避けるため、徐々に減速
                target_speed = speed_limit
                traci.vehicle.setSpeed(self.id, target_speed)
        except:
            pass

    # 予定経路に従い速度調整
    def executionDrive(self):
        if self.plannedPath:
            next_time = round(self.simTime + timeStep, 1)
            for i in range(len(self.plannedPath.t)):
                if self.plannedPath.t[i] == next_time:
                    nextpos_x = self.plannedPath.x[i]
                    nextpos_y = self.plannedPath.y[i]
                    dif_pos = np.sqrt(
                        (nextpos_x - self.pos_x) ** 2 + (nextpos_y - self.pos_y) ** 2
                    )
                    # 制限速度を考慮した速度計算
                    current_lane = f"{self.road}_{self.lane}"
                    speed_limit = traci.lane.getMaxSpeed(current_lane)
                    next_speed = max(min(dif_pos / timeStep, speed_limit), 0)
                    traci.vehicle.setSpeed(vehID=self.id, speed=next_speed)
                    break
        # 毎ステップで制限速度をチェック
        self.checkSpeedLimit()
