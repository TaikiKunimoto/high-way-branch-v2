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
        # not allow autonomous lanechange
        traci.vehicle.setLaneChangeMode(vehID=self.id, lcm=0b000000000000)
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

    # update own status every step
    def updateStatus(self, running_list):
        self.simTime = traci.simulation.getTime()

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

    # 予定経路を更新
    def updatePlannedPath(self, ob):

        if self.plannedPath:
            # check collision among own planned path and other PLANNED paths
            collision_list_planned, _ = self.checkPathCollision(
                self.plannedPath, self.receivedPaths, s_type="planned"
            )
            if len(collision_list_planned) > 0:
                if (
                    abs([-8.0, -4.8, -1.6][self.lane] - self.pos_y) > 0.5
                    and self.status == "lanechange"
                ):  # 車線変更開始後は止まらない
                    pass
                else:
                    print("my planned path collide")
                    self.status = None
                    self.isPathValid = False

            # check collision among own planned path and other DESIRED paths
            collision_list_desired, _ = self.checkPathCollision(
                self.plannedPath, self.receivedPaths, s_type="desired"
            )
            # 他車両の希望経路と重複している場合には、代替経路を生成
            if len(collision_list_desired) > 0 and self.status != "lanechange":
                self.judgeYield(collision_list_desired, ob)

        # 他の車両が全て交渉を受け入れる場合に代替経路を予定経路に設定
        if self.WithAgreementPhase and self.isWaitAgree:
            self.waitAgreeID.sort()
            self.receiveAgreeID.sort()
            print(
                "\tagree wait id", self.waitAgreeID, "receive id", self.receiveAgreeID
            )
            if self.waitAgreeID == self.receiveAgreeID:
                print("\talter to", self.waitPath.pathID)
                self.alter_straight_path.type = "planned"
                self.plannedPath = self.alter_straight_path
                self.alter_straight_path = None
                self.status = "yield"
                self.leader = self.waitPath.vehID
                self.leadPath = self.waitPath
                self.isPathValid = True

                self.isWaitAgree = False
                self.waitAgreeID = []
                self.receiveAgreeID = []
                self.waitPath = None

        # 期限切れの代替経路を削除
        if (
            self.alter_straight_path
            and round(self.simTime - self.alter_straight_path.t[0], 2) >= pathValidTime
        ):
            self.alter_straight_path = None
            self.isWaitAgree = False
            self.waitAgreeID = []
            self.receiveAgreeID = []
            self.waitPath = None

        # 新しい経路を生成
        if (
            not self.plannedPath
            or round(self.simTime - self.plannedPath.t[0], 2) >= pathValidTime
            or self.isPathValid == False
        ):

            # 経路候補をたくさん生成
            path_candidates = self.generatePlannedPath(ob=ob)

            # コスト関数が最小になるものを選定
            best_lanechange_path, best_straight_path = self.selectBestPath(
                path_candidates
            )

            if self.status == "lanechange":
                self.pathID += 1
                path_id = self.id + "_" + str(self.pathID)
                best_lanechange_path.pathID = path_id
                best_lanechange_path.vehID = self.id
                best_lanechange_path.type = "planned"

                self.plannedPath = best_lanechange_path

            else:
                self.pathID += 1
                path_id = self.id + "_" + str(self.pathID)
                best_straight_path.pathID = path_id
                best_straight_path.vehID = self.id
                best_straight_path.type = "planned"

                self.plannedPath = best_straight_path

            self.isPathValid = True

    def updateDesiredPath(self, ob):
        # 希望経路を決定 (最初の1ステップはスキップ)
        if self.simTime <= timeStep:
            return

        # 期限切れの経路を削除
        if (
            self.desiredPaths
            and round(self.simTime - self.desiredPaths[0].t[0], 2) >= pathValidTime
        ):
            self.desiredPaths = None

        if self.checkPathChangeNecessity() and not self.desiredPaths:
            # 経路候補をたくさん生成
            path_candidates = self.generateDesiredPath(ob=ob)
            # print("gen desired len", len(path_candidates))
            if len(path_candidates) == 0:
                self.desiredPaths = None
                return

            # コスト関数が最小になるものを選定
            best_change_path, _ = self.selectBestPath(path_candidates)

            if best_change_path:
                self.pathID += 1
                path_id = self.id + "_" + str(self.pathID)
                best_change_path.pathID = path_id
                best_change_path.vehID = self.id
                best_change_path.type = "desired"
                self.desiredPaths = [best_change_path]
            else:
                self.desiredPaths = None

    def checkRoutePriority(self, ego_path, x):

        # 同車線の後続車両と経路が重複していても先頭車両が優先
        if (
            x.vehID
            and traci.vehicle.getLaneIndex(x.vehID) == self.lane
            and x.x[0] < self.pos_x
        ):
            return True

        return False

    # 経路同士の重複判定
    def checkPathCollision(self, own_path, other_paths, s_type="all"):

        collision_list = []
        notCollision_list = []

        if len(other_paths) == 0:
            return collision_list, notCollision_list

        # 自身の経路は最新版、他車の経路は1ステップ古い
        for x in other_paths:
            if s_type == "all":
                pass
            elif x.type != s_type:
                continue

            startItr_own = None
            startItr_x = None
            for i in range(len(own_path.t)):
                if self.simTime == own_path.t[i]:
                    startItr_own = i
            for i in range(len(x.t)):
                if self.simTime == x.t[i]:
                    startItr_x = i

            if startItr_own is None or startItr_x is None:
                print(
                    own_path.pathID, own_path.t[0], "~", own_path.t[-1], file=sys.stderr
                )
                print(x.pathID, x.t[0], "~", x.t[-1], file=sys.stderr)

            # print("check collision", own_path.type, own_path.pathID, "and", x.type, x.pathID)
            assert own_path.t[startItr_own] == x.t[startItr_x]
            t_last = min(len(own_path.x) - startItr_own, len(x.x) - startItr_x) - 1

            isCollide = False
            dict = {
                "path": None,
                "collide_x": None,
                "collide_y": None,
                "speed": None,
                "duration": None,
            }

            # 同車線の後続車両と経路が重複していても先頭車両が優先
            if self.checkRoutePriority(own_path, x):
                notCollision_list.append(x)
                continue

            for i in range(1, t_last):
                # 車頭間隔を確保
                own_pos_x = own_path.x[startItr_own + i]
                own_pos_y = own_path.y[startItr_own + i]
                other_pos_x = x.x[startItr_x + i]
                other_pos_y = x.y[startItr_x + i]

                # estimate speed at each step
                own_dx = abs((own_pos_x - own_path.x[startItr_own + i - 1]) / timeStep)
                own_dy = abs((own_pos_y - own_path.y[startItr_own + i - 1]) / timeStep)
                other_dx = abs((other_pos_x - x.x[startItr_x + i - 1]) / timeStep)
                other_dy = abs((other_pos_y - x.y[startItr_x + i - 1]) / timeStep)

                if own_pos_x < other_pos_x:
                    safe_distx = max(self.length, own_dx * tau)
                else:
                    safe_distx = max(self.length, other_dx * tau)
                safe_disty = self.width

                if (
                    abs(own_pos_x - other_pos_x) < safe_distx
                    and abs(own_pos_y - other_pos_y) < safe_disty
                ):
                    isCollide = True

                if isCollide:
                    # print("collide at", own_path.pathID, own_path.type, "other", x.pathID, x.type, "time", x.t[startItr_x+i],\
                    #     "own_pos (", own_pos_x, ",", own_pos_y, ") other_pos (", other_pos_x, ",", other_pos_y, ")")

                    dict["path"] = x
                    dict["collideID"] = x.vehID
                    dict["collide_x"] = other_pos_x
                    dict["collide_y"] = other_pos_y
                    dict["speed"] = other_dx
                    dict["duration"] = x.t[startItr_x + i] - self.simTime
                    break

            if isCollide:
                collision_list.append(dict)
            else:
                notCollision_list.append(x)

        return collision_list, notCollision_list

    # 経路変更の必要があるか判定
    def checkPathChangeNecessity(self):

        if (
            self.status not in ["yield", "lanechange"]
            and self.lane == 0
            and self.pos_x >= mergeStartPos
        ):
            return True
        else:
            return False

    # 他車両に道を譲るか判定
    def judgeYield(self, collision_list, ob):

        desired_list = []
        for i in collision_list:
            desired_list.append(i["path"].pathID)
        print(
            "Negotiation list",
            desired_list,
            "isWaitAgree",
            self.isWaitAgree,
            "waitPath",
            self.waitPath,
        )

        if self.WithAgreementPhase:
            if self.isWaitAgree and self.waitPath.pathID in desired_list:
                # 他車両との合意待ち中は他との交渉は行わない
                return
            elif self.isWaitAgree and self.waitPath.pathID not in desired_list:
                # 合意待ち中に元の希望経路が無くなった場合，一から交渉を始める
                self.isWaitAgree = False
                self.waitAgreeID = []
                self.receiveAgreeID = []
                self.waitPath = None

        for path in collision_list:
            leadPath = path["path"]

            # 同車線の車両の場合には無視（先頭の車両が優先権を持つ）
            if traci.vehicle.getLaneIndex(leadPath.vehID) == self.lane:
                print("reject because same_lane")
                continue

            if (
                self.pos_x > 300
            ):  # 合流地点を通過した車両は譲ることができないため判定をしない
                return

            # 一度拒否した希望経路は経路が変わるまで判定しない
            if leadPath.pathID in self.yetJudgedList:
                print("reject because same_path")
                continue

            print("judge yield to", leadPath.vehID)

            # 代替経路を生成
            alterPaths = self.generateAlterPath(path, ob)
            print("gen alter path", len(alterPaths))

            if len(alterPaths) == 0:
                print("cannot generate alter paths")
                self.yetJudgedList.append(leadPath.pathID)
                self.count_refuse_physics += 1
                continue

            else:
                self.alter_change_path, self.alter_straight_path = self.selectBestPath(
                    alterPaths
                )

                # if self.alter_change_path:
                #     # Cascading 対策
                #     collision_list, _  = self.checkPathCollision(self.alter_change_path, self.receivedPaths, s_type="planned")
                #     if len(collision_list) != 0:
                #         self.alter_change_path = None

                # 経路変更による車速の増減を比較
                if self.alter_straight_path:
                    utility = 0
                    for path in self.receivedPaths:
                        if path.vehID == leadPath.vehID and path.type == "planned":
                            # 経路変更による速度の増加量

                            if consider_3s_later:
                                alter_straight_path_3s = self.alter_straight_path.s_d[
                                    :30
                                ]
                                d_ego_speed = min(alter_straight_path_3s) - maxSpeed
                                d_alt_speed = leadPath.s_d[30] - path.s_d[30]
                                print(
                                    "d_ego_speed = ",
                                    min(alter_straight_path_3s),
                                    " - ",
                                    maxSpeed,
                                )
                                print(
                                    "d_alt_speed = ",
                                    leadPath.s_d[30],
                                    " - ",
                                    path.s_d[30],
                                )
                            else:
                                d_ego_speed = (
                                    min(self.alter_straight_path.s_d) - maxSpeed
                                )
                                d_alt_speed = (
                                    leadPath.s_d[-1] - path.s_d[-1]
                                )  # 最終速度の増加量
                                print(
                                    "d_ego_speed = ",
                                    min(self.alter_straight_path.s_d),
                                    " - ",
                                    maxSpeed,
                                )
                                print(
                                    "d_alt_speed = ",
                                    leadPath.s_d[-1],
                                    " - ",
                                    path.s_d[-1],
                                )
                                print("if consider 3s later")
                                print(
                                    "d_alt_speed = ",
                                    leadPath.s_d[30],
                                    " - ",
                                    path.s_d[30],
                                )

                            # 経路変更による減速度の改善量
                            alter_decel = 0.0
                            ego_decel = 0.0
                            desire_decel = 0.0
                            alt_decel = 0.0

                            # 経路変更により強いられる減速の絶対値和
                            for i in self.alter_straight_path.s_dd:
                                if i < 0:
                                    alter_decel += i
                            for i in self.plannedPath.s_dd:
                                if i < 0:
                                    ego_decel += i
                            for i in leadPath.s_dd:
                                if i < 0:
                                    desire_decel += i
                            for i in path.s_dd:
                                if i < 0:
                                    alt_decel += i

                            # d_ego_speed = round((d_ego_speed / maxSpeed), 2)
                            if divide_DT:
                                if consider_3s_later:
                                    d_alt_speed = round(
                                        (d_alt_speed / leadPath.s_d[30]), 2
                                    )
                                else:
                                    d_alt_speed = round(
                                        (d_alt_speed / leadPath.s_d[-1]), 2
                                    )
                            # else:
                            #    d_alt_speed = round((d_alt_speed / maxSpeed), 2)

                            # 後続の車両台数
                            followVehs_alt_list = []
                            followVehs_ego_list = []
                            followCoefficient1_alt_list = []
                            followCoefficient1_ego_list = []
                            followCoefficient2_alt_list = []
                            followCoefficient2_ego_list = []
                            followCoefficient2up_alt_list = []
                            followCoefficient2up_ego_list = []
                            if all_follow_Range:
                                for i in self.receivedPaths:
                                    if (
                                        i.x[0] < self.pos_x
                                        and abs(i.y[0] - self.pos_y) < 1.0
                                    ):
                                        co_ego1 = (
                                            commRange - self.pos_x + i.x[0]
                                        ) / commRange
                                        co_ego2 = (
                                            commRange - self.pos_x + i.x[0]
                                        ) ** 2 / commRange**2
                                        co_ego2up = (
                                            (commRange) ** 2
                                            - (self.pos_x - i.x[0]) ** 2
                                        ) / commRange**2
                                        for j in followVehs_ego_list:
                                            if j == i.vehID:
                                                continue
                                        followVehs_ego_list.append(i.vehID)
                                        followCoefficient1_ego_list.append(co_ego1)
                                        followCoefficient2_ego_list.append(co_ego2)
                                        followCoefficient2up_ego_list.append(co_ego2up)
                                    if (
                                        i.x[0] < path.x[0]
                                        and abs(i.y[0] - path.y[0]) < 1.0
                                    ):
                                        for j in followVehs_alt_list:
                                            if j == i.vehID:
                                                continue
                                        followVehs_alt_list.append(i.vehID)
                                        co_alt1 = (
                                            commRange - path.x[0] + i.x[0]
                                        ) / commRange
                                        co_alt2 = (
                                            commRange - path.x[0] + i.x[0]
                                        ) ** 2 / commRange**2
                                        co_alt2up = (
                                            (commRange) ** 2 - (path.x[0] - i.x[0]) ** 2
                                        ) / commRange**2
                                        followCoefficient1_alt_list.append(co_alt1)
                                        followCoefficient2_alt_list.append(co_alt2)
                                        followCoefficient2up_alt_list.append(co_alt2up)
                                    if (
                                        abs(i.x[0] - self.pos_x) < collision_distance
                                        and abs(i.y[0] - self.pos_y) < 1.0
                                    ):
                                        self.collision_counter += 1
                                    if (
                                        abs(i.x[0] - self.pos_x) < caution_distance
                                        and abs(i.y[0] - self.pos_y) < 1.0
                                    ):
                                        self.caution_counter += 1
                            else:
                                for i in self.receivedPaths:  # commRange=200
                                    if (
                                        (self.pos_x - i.x[0]) < follow_range
                                        and i.x[0] < self.pos_x
                                        and abs(i.y[0] - self.pos_y) < 1.0
                                    ):
                                        co_ego1 = (
                                            commRange - self.pos_x + i.x[0]
                                        ) / commRange
                                        co_ego2 = (
                                            commRange - self.pos_x + i.x[0]
                                        ) ** 2 / commRange**2
                                        co_ego2up = (
                                            (commRange) ** 2
                                            - (self.pos_x - i.x[0]) ** 2
                                        ) / commRange**2
                                        for j in followVehs_ego_list:
                                            if j != i.vehID:
                                                continue
                                            co_ego1 = 0
                                            co_ego2 = 0
                                            co_ego2up = 0
                                            break
                                        followVehs_ego_list.append(i.vehID)
                                        followCoefficient1_ego_list.append(co_ego1)
                                        followCoefficient2_ego_list.append(co_ego2)
                                        followCoefficient2up_ego_list.append(co_ego2up)
                                    if (
                                        (path.x[0] - i.x[0]) < follow_range
                                        and i.x[0] < path.x[0]
                                        and abs(i.y[0] - path.y[0]) < 1.0
                                    ):
                                        co_alt1 = (
                                            commRange - path.x[0] + i.x[0]
                                        ) / commRange
                                        co_alt2 = (
                                            commRange - path.x[0] + i.x[0]
                                        ) ** 2 / commRange**2
                                        co_alt2up = (
                                            (commRange) ** 2 - (path.x[0] - i.x[0]) ** 2
                                        ) / commRange**2
                                        for j in followVehs_alt_list:
                                            if j != i.vehID:
                                                continue
                                            co_alt1 = 0
                                            co_alt2 = 0
                                            co_alt2up = 0
                                            break
                                        followVehs_alt_list.append(i.vehID)
                                        followCoefficient1_alt_list.append(co_alt1)
                                        followCoefficient2_alt_list.append(co_alt2)
                                        followCoefficient2up_alt_list.append(co_alt2up)
                                    if (
                                        abs(i.x[0] - self.pos_x) < collision_distance
                                        and abs(i.y[0] - self.pos_y) < 1.0
                                    ):
                                        self.collision_counter += 1
                                    if (
                                        abs(i.x[0] - self.pos_x) < caution_distance
                                        and abs(i.y[0] - self.pos_y) < 1.0
                                    ):
                                        self.caution_counter += 1

                            print(followVehs_ego_list)
                            print(followVehs_alt_list)
                            followVehs_ego = len(set(followVehs_ego_list))
                            followVehs_alt = len(set(followVehs_alt_list))

                            followCoefficient1_ego = sum(followCoefficient1_ego_list)
                            followCoefficient1_alt = sum(followCoefficient1_alt_list)
                            followCoefficient2_ego = sum(followCoefficient2_ego_list)
                            followCoefficient2_alt = sum(followCoefficient2_alt_list)
                            followCoefficient2up_ego = sum(
                                followCoefficient2up_ego_list
                            )
                            followCoefficient2up_alt = sum(
                                followCoefficient2up_alt_list
                            )

                            print(followCoefficient1_ego)
                            print(followCoefficient1_alt)
                            print(followCoefficient2_ego)
                            print(followCoefficient2_alt)
                            print(followCoefficient2up_ego)
                            print(followCoefficient2up_alt)

                            k1 = 1
                            # kato's function
                            # utility = self.alpha*(d_ego_speed*(1+followVehs_ego)) + (1-self.alpha)*(d_alt_speed)

                            # original function
                            # utility = self.alpha*(d_ego_speed*(1+followVehs_ego)) + (1-self.alpha)*(d_alt_speed)*(1+followVehs_alt)

                            # improvement function (linear)
                            # utility = d_ego_speed*(1+followCoefficient1_ego) + d_alt_speed*(1+followCoefficient1_alt)

                            # improvement function (quadratic / 下に凸)
                            # utility = d_ego_speed*(1+followCoefficient2_ego) + d_alt_speed*(1+followCoefficient2_alt)

                            # improvement function (quadratic / 上に凸)
                            utility = d_ego_speed * (
                                1 + followCoefficient2up_ego
                            ) + d_alt_speed * (1 + followCoefficient2up_alt)

                            # PriMa's function (level : low)
                            # utility = 0.2 - (self.plannedPath.s_d[1] - min(self.alter_straight_path.s_d))/self.plannedPath.s_d[1]

                            # PriMa's function (level : medium)
                            # utility = 0.4 - (self.plannedPath.s_d[1] - min(self.alter_straight_path.s_d))/self.plannedPath.s_d[1]

                            # always accept
                            # utility = 1

                            print(self.id, "culculate utility to ", path.vehID)
                            print(
                                "vehID:",
                                self.id,
                                "utility:",
                                utility,
                                "d_ego:",
                                d_ego_speed,
                                "d_alt",
                                d_alt_speed,
                            )
                            print(
                                "follow_ego :",
                                followVehs_ego,
                                "follow_alt(merge) :",
                                followVehs_alt,
                            )

                    # 両車両の加速度の増減が閾値以上であれば道を譲る
                    if utility > yield_th or (
                        self.status == "yield" and self.leader == leadPath.vehID
                    ):
                        self.count_accept_utility += 1

                        self.pathID += 1
                        path_id = self.id + "_" + str(self.pathID)
                        self.alter_straight_path.pathID = path_id
                        self.alter_straight_path.vehID = self.id

                        if self.WithAgreementPhase:
                            self.alter_straight_path.type = "alter"
                            self.alter_straight_path.yieldTo = leadPath.vehID

                            # 他の車両の交渉受け入れを待つ
                            self.waitAgreeID = leadPath.negotiationID
                            self.receiveAgreeID.append(self.id)
                            self.isWaitAgree = True
                            self.waitPath = leadPath
                            return

                        else:
                            self.alter_straight_path.type = "planned"
                            self.plannedPath = self.alter_straight_path
                            self.alter_straight_path = None
                            self.status = "yield"
                            self.leader = leadPath.vehID
                            self.leadPath = leadPath
                            print("alter to", leadPath.pathID)
                            self.isPathValid = True

                    else:
                        self.yetJudgedList.append(leadPath.pathID)
                        self.count_refuse_utility += 1

    def sendMCM(self, central_path_server):
        # send PLANNED path
        if self.plannedPath:
            self.count_send_PT += 1
            # 経路がmaxPathLen秒以上ある場合にはmaxT秒分だけ送る
            if len(self.plannedPath.t) > round(maxPathLen / timeStep):
                count = int(round(maxPathLen / timeStep)) + 1
                tmp = copy.deepcopy(self.plannedPath)
                tmp.t = tmp.t[:count]
                tmp.d = tmp.d[:count]
                tmp.d_d = tmp.d_d[:count]
                tmp.d_dd = tmp.d_dd[:count]
                tmp.d_ddd = tmp.d_ddd[:count]
                tmp.s = tmp.s[:count]
                tmp.s_d = tmp.s_d[:count]
                tmp.s_dd = tmp.s_dd[:count]
                tmp.s_ddd = tmp.s_ddd[:count]
                tmp.x = tmp.x[:count]
                tmp.y = tmp.y[:count]

                central_path_server.append(tmp)

            else:
                central_path_server.append(self.plannedPath)

        # send DESIRED path
        if self.desiredPaths:
            for dp in self.desiredPaths:
                self.count_send_DT += 1
                central_path_server.append(dp)

        # send ALTER path
        if self.alter_straight_path:
            self.count_send_AT += 1
            cp = copy.deepcopy(self.alter_straight_path)
            central_path_server.append(cp)

    def receiveMCM(self, central_path_server):
        # 通信範囲内の他車両の経路情報を受信
        self.receivedPaths = [
            path
            for path in central_path_server
            if (
                path.vehID != self.id
                and np.sqrt(
                    (self.pos_x - path.x[0]) ** 2 + (self.pos_y - path.y[0]) ** 2
                )
                <= commRange
            )
        ]

        # 他の車両が交渉に合意しているかをチェック
        if self.WithAgreementPhase and self.isWaitAgree:
            for path in self.receivedPaths:
                if (
                    path.type == "alter"
                    and path.yieldTo == self.alter_straight_path.yieldTo
                ):
                    self.receiveAgreeID.append(path.vehID)

    # 経路を10秒後まで延長
    def extendPath(self, path):
        count = int(round((self.simTime - path.t[0]) / timeStep))
        lack = 101 - (len(path.t) - count)

        path.t = path.t[count:]
        path.t.extend([round(path.t[-1] + timeStep * (i + 1), 3) for i in range(lack)])
        path.s_d = path.s_d[count:]
        path.s_d.extend([path.s_d[-1] for i in range(lack)])
        path.s_dd.extend([path.s_dd[-1] for i in range(lack)])
        path.s_ddd.extend([path.s_ddd[-1] for i in range(lack)])
        path.x = path.x[count:]
        path.x.extend(
            [path.x[-1] + (path.x[-1] - path.x[-2]) * (i + 1) for i in range(lack)]
        )
        path.y = path.y[count:]
        path.y.extend(
            [path.y[-1] + (path.y[-1] - path.y[-2]) * (i + 1) for i in range(lack)]
        )

        path.convergeTime = max(path.convergeTime - count * timeStep, 0)

        return path

    # 予定経路を計算
    def generatePlannedPath(self, ob):
        path_candidates = []

        if self.status == "lanechange":
            print("extend path")
            # 予定経路を延長
            self.plannedPath = self.extendPath(self.plannedPath)
            return [self.plannedPath]

        if self.status == "yield":
            print("extend path")
            # 予定経路を延長
            path_candidates = [self.extendPath(self.plannedPath)]

            realLeader = traci.vehicle.getLeader(self.id)
            if realLeader and self.leader != realLeader[0]:
                leadPath = [
                    path
                    for path in self.receivedPaths
                    if path.vehID == realLeader[0] and path.type == "planned"
                ]
                if leadPath:
                    _, path_candidates = self.checkPathCollision(
                        leadPath[0], path_candidates
                    )

            if len(path_candidates) > 0:
                return path_candidates
            elif realLeader:
                self.leader = realLeader[0]

        if self.leader:  # 前方車両に追従走行
            center_lines = [-8.0, -4.8, -1.6]
            # 前方車両が既に停止している場合
            if traci.vehicle.getSpeed(self.leader) < 0.1:

                target_pos = (
                    traci.vehicle.getPosition(self.leader)[0] - D_0 - self.length
                )
                if self.leader == "stone":
                    target_pos = target_pos - D_0
                path_candidates = generate_frenet_frame(
                    objects=ob,
                    c_s_d=self.speed,
                    c_s_dd=self.accel,
                    c_s=self.pos_x,
                    c_d=self.pos_y,
                    target_pos=target_pos,
                    target_speed=0,
                    mode="stop",
                    route=self.route,
                    simTime=self.simTime,
                    center_lines=[center_lines[self.lane]],
                    vehInstance=self,
                )

                # print("A gen stop len", len(path_candidates))
                if len(path_candidates) > 0:
                    self.status = "stop"
                    return path_candidates

            else:
                leadPath = [
                    path
                    for path in self.receivedPaths
                    if path.vehID == self.leader and path.type == "planned"
                ]
                if leadPath:
                    leadPath = self.extendPath(leadPath[0])

                    # 前方車両が停止する場合
                    for index, value in enumerate(leadPath.s_d):
                        if value == 0.0:
                            converge = index

                            target_pos = leadPath.x[converge] - D_0 - self.length
                            path_candidates = generate_frenet_frame(
                                objects=ob,
                                c_s_d=self.speed,
                                c_s_dd=self.accel,
                                c_s=self.pos_x,
                                c_d=self.pos_y,
                                target_pos=target_pos,
                                target_speed=0,
                                mode="stop",
                                route=self.route,
                                simTime=self.simTime,
                                center_lines=[center_lines[self.lane]],
                                vehInstance=self,
                            )

                            _, path_candidates = self.checkPathCollision(
                                leadPath, path_candidates
                            )
                            print("B gen stop len", len(path_candidates))

                            if len(path_candidates) > 0:
                                self.status = "stop"
                                return path_candidates

                            break

                    duration = round(leadPath.t[-1] - self.simTime, 2)
                    target_speed = leadPath.s_d[-1]
                    target_pos = leadPath.x[-1]
                    center_lines = [-8.0, -4.8, -1.6]
                    c_s = self.pos_x

                    # 安全のためのマージン
                    target_pos = target_pos - max(D_0 + self.length, target_speed * tau)
                    path_candidates = generate_frenet_frame(
                        objects=ob,
                        c_s=c_s,
                        c_s_d=self.speed,
                        c_s_dd=self.accel,
                        c_d=center_lines[self.lane],
                        target_pos=target_pos,
                        target_speed=maxSpeed,
                        mode="free",
                        route=self.route,
                        center_lines=[center_lines[self.lane]],
                        simTime=self.simTime,
                        vehInstance=self,
                    )
                    # 上のtarget_speedはmaxSpeedではなくtarget_speedなのでは？

                    _, path_candidates = self.checkPathCollision(
                        leadPath, path_candidates
                    )
                    # print("gen follow to", self.leader ,"len", len(path_candidates))

                    if len(path_candidates) > 0:
                        self.status = "follow"
                        return path_candidates

        # 停止、追従経路が生成できない場合には追いつけないものと判断し自由走行を行う
        if len(path_candidates) == 0:
            self.status = "free"
            center_lines = [-8.0, -4.8, -1.6]
            path_candidates = generate_frenet_frame(
                objects=ob,
                c_s=self.pos_x,
                c_s_d=self.speed,
                c_s_dd=self.accel,
                c_d=self.pos_y,
                target_speed=maxSpeed,
                mode="free",
                route=self.route,
                center_lines=[center_lines[self.lane]],
                simTime=self.simTime,
                vehInstance=self,
            )

            if self.leadPath:
                _, path_candidates = self.checkPathCollision(
                    self.leadPath, path_candidates
                )

            if len(path_candidates) > 0:
                pass
                # print("B gen free len", len(path_candidates))
            else:
                path_candidates = emergency_stop(
                    self.pos_x, self.speed, self.pos_y, self.route
                )
                # print('Emergency Brake time', self.simTime, "veh", self.id, file=sys.stderr)
                self.status == "stop"
                for path in path_candidates:
                    path.t = [round(n + self.simTime, 1) for n in path.t]

        if len(path_candidates) == 0:
            path_candidates = emergency_stop(
                self.pos_x, self.speed, self.pos_y, self.route
            )

        return path_candidates

    def generateDesiredPath(self, ob):
        center_lines = [[-4.8], [-1.6, -8.0], [-4.8]]

        if self.plannedPath:
            count = int(round((self.simTime - self.plannedPath.t[0]) / timeStep))
            c_d_d = self.plannedPath.d_d[count]
            c_d_dd = self.plannedPath.d_dd[count]
        else:
            c_d_d = 0.0
            c_d_dd = 0.0

        path_candidates = generate_frenet_frame(
            objects=ob,
            c_s=self.pos_x,
            c_s_d=self.speed,
            c_s_dd=self.accel,
            c_d=self.pos_y,
            c_d_d=c_d_d,
            c_d_dd=c_d_dd,
            target_speed=maxSpeed,
            mode="free",
            route=self.route,
            center_lines=center_lines[self.lane],
            simTime=self.simTime,
            vehInstance=self,
        )
        for path in path_candidates:
            path.isLaneChange = True

        return path_candidates

    # 代替経路を計算
    def generateAlterPath(self, path, ob):

        collide_path = path["path"]
        target_pos = path["collide_x"]
        target_speed = path["speed"]
        duration = round(path["duration"], 2)

        # 減速して対象車両の後方に着く
        alterPaths = generate_frenet_frame(
            objects=ob,
            c_s=self.pos_x,
            c_s_d=self.speed,
            c_s_dd=self.accel,
            c_d=self.pos_y,
            target_pos=target_pos,
            target_speed=target_speed,
            route=self.route,
            mode="yield",
            duration=duration,
            simTime=self.simTime,
            vehInstance=self,
        )

        # 代替経路がしっかりと希望経路との衝突を避けられているかを判定
        _, alterPaths = self.checkPathCollision(collide_path, alterPaths)
        frontVeh = traci.vehicle.getLeader(self.id)
        print(self.id, "'s frontVeh : ", frontVeh)
        if frontVeh:
            leadPath = [
                path
                for path in self.receivedPaths
                if path.vehID == frontVeh[0] and path.type == "planned"
            ]
            if leadPath:
                _, alterPaths = self.checkPathCollision(leadPath[0], alterPaths)

        return alterPaths

    # コスト関数が最小になる経路を計算
    def selectBestPath(self, path_candidates):
        # find minimum cost path
        min_change_cost = float("inf")
        min_straight_cost = float("inf")
        best_change_path = None
        best_straight_path = None

        for fp in path_candidates:
            # 車線変更を伴う経路のうち最善
            if fp.isLaneChange == True and min_change_cost >= fp.cf:
                min_change_cost = fp.cf
                best_change_path = fp
            # 直進する経路のうち最善
            elif fp.isLaneChange == False and min_straight_cost >= fp.cf:
                min_straight_cost = fp.cf
                best_straight_path = fp

        return best_change_path, best_straight_path

    # TODO errorを治す

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
                # 急ブレーキを避けるため、徐々に減速
                target_speed = min(self.speed * 0.9, speed_limit)
                traci.vehicle.setSpeed(self.id, target_speed)
        except:
            pass

    # 予定経路に従い速度調整
    def executionDrive(self):
        print("vehID:", self.id, "road:", self.road, "lane:", self.lane)
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

    def getStatistics(self):
        return (
            self.count_accept_utility,
            self.count_refuse_utility,
            self.count_refuse_physics,
            self.count_send_PT,
            self.count_send_DT,
            self.count_send_AT,
        )

    # 周辺車両との距離を考慮し車線変更できるか判定
    def judgeAndDoLaneChange(self):
        dif_y = abs(self.pos_y - self.plannedPath.y[-1])
        if dif_y != 0 and dif_y < LANE_WIDTH / 2:
            traci.vehicle.changeLane(vehID=self.id, laneIndex=1, duration=0)

    def returnCollisionTime(self):
        return self.collision_counter

    def returnCautionTime(self):
        return self.caution_counter

    def returnParameter(self):
        if consider_3s_later:
            if divide_DT:
                return 300001
            else:
                return 300000
        else:
            if divide_DT:
                return 100001
            else:
                return 100000

    def returnFollowRange(self):
        return follow_range

    # destructor
    def __del__(self):
        print("delete veh:", self.id)
