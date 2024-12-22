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
minGap = 2.5  # [m]
reactionTime = 0.75  # [s]
frictionCoefficient = 0.7  # 摩擦係数
LANE_WIDTH = 3.2  # [m]
LANE_CHANGE_MARGIN = 400.0  # [m] 渋滞発生地点の何メートル手前から車線変更を許可するか
SPEED_IMPROVEMENT_THRESHOLD = 10.0  # 車線変更による速度改善の閾値 [%]

timeStep = 0.1  # [s]

vehicle_instances = {}  # グローバルな車輌管理辞書


class CustomCAV:
    # constructor
    def __init__(self, vehID, alpha, withAgree=False):
        self.id = str(vehID)
        vehicle_instances[self.id] = self

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
        self.lane_change_status = "unavailable"  # available, unavailable
        self.is_yielding = False  # 協調中かどうか
        self.is_lane_changing = False  # 車線変更中かどうか
        self.receiving_cooperative_from_id = None  # 協調中に譲ってもらう車両のID
        self.providing_cooperative_to_id = None  # 協調して譲る車両のID
        self.action = "stay"  # stay, change_left, change_right
        self.priority = 6  # 1(high), 2, 3, 4, 5(low), 6(None)
        self.leader_distance = None  # 前方車両との距離
        self.leader_speed = None  # 前方車両の速度
        self.left_followers = None  # 左後続車両
        self.right_followers = None  # 右後続車両
        self.left_leaders = None  # 左前方車両
        self.right_leaders = None  # 右前方車両

        self.pos_x = 0
        self.pos_y = 0
        self.angle = None
        self.speed = 0
        self.accel = 0
        self.leader = None
        self.length = 5.0
        self.width = 1.8
        self.reaction_distance = 0  # 空走距離 [m]
        self.breaking_distance = 0  # 制動距離 [m]
        self.safety_gap = self.reaction_distance + self.breaking_distance + minGap

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
            (2, "r_exit"): {"action": "stay", "priority": 5, "conditions": []},
            (2, "r_pass"): {
                "action": "change_right",
                "priority": 4,
                "conditions": [
                    lambda: self.lane_change_status == "available",
                    lambda: self._isPredictedSpeedIncrease("right"),
                ],
            },
            # Lane 1のルール
            (1, "r_exit"): {
                "action": "change_left",
                "priority": 2,
                "conditions": [
                    lambda: self.lane_change_status == "available",
                ],
            },
            (1, "r_pass"): {
                "action": "change_right",
                "priority": 3,
                "conditions": [
                    lambda: self.lane_change_status == "available",
                    lambda: self._isPredictedSpeedIncrease("right"),
                ],
            },
            # Lane 0のルール
            (0, "r_exit"): {
                "action": "change_left",
                "priority": 1,
                "conditions": [
                    lambda: self.lane_change_status == "available",
                ],
            },
            (0, "r_pass"): {"action": "stay", "priority": 5, "conditions": []},
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
        self.leader_distance = self.leader[1] if self.leader is not None else None
        self.leader_speed = (
            traci.vehicle.getSpeed(self.leader[0]) if self.leader is not None else None
        )
        self._calculateSafetyGap()

        # 車線変更が可能なポイントを通過したら車線変更を可能にする
        if self.lane_change_status == "unavailable":
            if self.hasPassedLaneChangePoint(congestion_point):
                self.lane_change_status = "available"

        if self.lane_change_status == "available":
            if self.road != "MainLane1":
                self.lane_change_status = "unavailable"

        self._getFollowerAndLeaderRunningAnotherLane()

    # 適切な車間距離の計算
    def _calculateSafetyGap(self):
        speed_kmh = self.speed * 3.6
        # 空走距離
        self.reaction_distance = self.speed * reactionTime
        # 制動距離
        self.breaking_distance = (speed_kmh**2) / (254.016 * frictionCoefficient)
        # 安全距離
        self.safety_gap = self.reaction_distance + self.breaking_distance + minGap

    # 後続車輌と先行車輌を取得する
    def _getFollowerAndLeaderRunningAnotherLane(self):
        self._resetFollowerAndLeaderVehicles()

        if self.road is None or self.lane is None or self.road != "MainLane1":
            return

        own_position = traci.vehicle.getLanePosition(self.id)

        # レーン番号に基づいて確認すべき隣接レーンを決定
        check_lanes = []
        if self.lane == 0:
            check_lanes.append(("left", 1))  # レーン0の場合、左側のレーン1をチェック
        elif self.lane == 1:
            check_lanes.append(("right", 0))  # レーン1の場合、右側のレーン0をチェック
            check_lanes.append(("left", 2))  # レーン1の場合、左側のレーン2もチェック
        elif self.lane == 2:
            check_lanes.append(("right", 1))  # レーン2の場合、右側のレーン1をチェック

        for direction, lane_num in check_lanes:
            lane_id = f"{self.road}_{lane_num}"
            lane_vehicles = traci.lane.getLastStepVehicleIDs(lane_id)
            if not lane_vehicles:
                continue

            followers = []
            leaders = []

            for vehicle_id in lane_vehicles:
                if vehicle_id == self.id:
                    continue

                vehicle_position = traci.vehicle.getLanePosition(vehicle_id)
                distance = abs(own_position - vehicle_position)

                if vehicle_position < own_position:
                    followers.append((vehicle_id, distance))
                else:
                    leaders.append((vehicle_id, distance))

            followers = sorted(followers, key=lambda x: x[1])
            leaders = sorted(leaders, key=lambda x: x[1])

            if direction == "left":
                self.left_followers = followers
                self.left_leaders = leaders
            elif direction == "right":
                self.right_followers = followers
                self.right_leaders = leaders

    def _resetFollowerAndLeaderVehicles(self):
        self.left_followers = None
        self.right_followers = None
        self.left_leaders = None
        self.right_leaders = None

    # 車線変更が可能なポイントを通過したかどうか
    def hasPassedLaneChangePoint(self, congestion_point):
        lane_length = traci.lane.getLength("MainLane1_2")
        current_pos = traci.vehicle.getLanePosition(self.id)

        if congestion_point is None:
            merge_start_pos = lane_length - LANE_CHANGE_MARGIN
        else:
            merge_start_pos = congestion_point - LANE_CHANGE_MARGIN

        if current_pos > merge_start_pos:
            return True
        return False

    # 車両の速度を調整
    def controlSpeed(self):
        # 協調フェーズの場合はここで速度制御しない
        if self.is_yielding == True or self.is_lane_changing == True:
            return

        # 無効な道路上の場合は制御しない
        if self.road is None or self.road.startswith(":"):
            return

        # 現在のレーンと制限速度を取得
        current_lane = f"{self.road}_{self.lane}"
        speed_limit = traci.lane.getMaxSpeed(current_lane)

        # 前方車両がいない場合
        if self.leader is None:
            self._controlSpeedBySpeedLimit(speed_limit)
            return

        # 前方車両がいる場合
        else:
            speed_diff = self.speed - self.leader_speed
            min_duration = self._calculateSafeDecelDuration(speed_diff)
            ttc_with_safety_margin = self._calculateTTC(
                self.leader_distance + minGap, speed_diff
            )
            # 前方車両との距離 > safety_gap の場合
            if self.leader_distance > self.safety_gap:
                if speed_diff <= 0:
                    self._controlSpeedBySpeedLimit(speed_limit)
                    return
                elif min_duration < ttc_with_safety_margin:
                    self._controlSpeedBySpeedLimit(speed_limit)
                    return
                else:
                    # 通常の減速
                    duration = min(ttc_with_safety_margin, min_duration)
                    traci.vehicle.slowDown(self.id, self.leader_speed, duration)
                    return

            # 前方車両との距離 = safety_gap の場合
            elif self.leader_distance == self.safety_gap:
                if speed_diff > 0:
                    # 通常の減速
                    duration = min(ttc_with_safety_margin, min_duration)
                    traci.vehicle.slowDown(self.id, self.leader_speed, duration)
                    return
                else:
                    return

            # 前方車両との距離 < safety_gap の場合
            else:
                if speed_diff >= 0:
                    if self.leader_distance < minGap:
                        # 急ブレーキ
                        self._emergencyBreak(self.leader_speed)
                        return
                    else:
                        # 通常の減速
                        duration = min(ttc_with_safety_margin, min_duration)
                        traci.vehicle.slowDown(self.id, self.leader_speed, duration)
                else:
                    return

    # 制限速度に基づいて速度を調整
    def _controlSpeedBySpeedLimit(self, speed_limit):
        # 減速
        if self.speed > speed_limit:
            safe_duration = self._calculateSafeDecelDuration(self.speed - speed_limit)
            traci.vehicle.slowDown(self.id, speed_limit, safe_duration)
            return
        # 維持
        elif self.speed == speed_limit:
            return
        # 加速
        else:
            safe_duration = self._calculateSafeAccelDuration(speed_limit - self.speed)
            traci.vehicle.slowDown(self.id, speed_limit, safe_duration)
            return

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
            return math.inf
        return distance / speed_diff

    # 衝突回避のための速度調整
    def _emergencyBreak(self, targetSpeed):
        # print("emergency brake")
        self.emergency_brake_counter += 1
        traci.vehicle.setSpeed(self.id, targetSpeed)

    # 車線変更を実行
    def executeLaneChange(self):
        # TODO 衝突が発生しなくなったらここの条件をなくす
        if self.road != "MainLane1":
            return

        if self.action == "stay":
            return

        direction = "left" if self.action == "change_left" else "right"
        lane_change_amount = 1 if direction == "left" else -1

        if self._canChangeLane(direction):
            # 車線変更が可能な場合は実行
            traci.vehicle.changeLane(self.id, self.lane + lane_change_amount, 0)
            self._resetLaneChangeState()
        else:
            if self.receiving_cooperative_from_id in vehicle_instances:
                supporting_vehicle = vehicle_instances[
                    self.receiving_cooperative_from_id
                ]
                supporting_vehicle_pos = traci.vehicle.getLanePosition(
                    supporting_vehicle.id
                )
                own_pos = traci.vehicle.getLanePosition(self.id)
                position_diff = own_pos - supporting_vehicle_pos
            else:
                position_diff = -1 * math.inf

            if self.receiving_cooperative_from_id is None or position_diff < -10:
                # 車線変更ができず、まだ協調車両がいない場合
                self._decideYieldingVehicle()
                self._requestCooperation()
            # 協調車輌と自身の速度を調整
            self._adjustSpeedForCooperation()

    # 協調車輌に協調を要求
    def _requestCooperation(self):
        if self.receiving_cooperative_from_id in vehicle_instances:
            supporting_vehicle = vehicle_instances[self.receiving_cooperative_from_id]
            supporting_vehicle.is_yielding = True
            supporting_vehicle.providing_cooperative_to_id = self.id

    # 協調車輌と自身の速度を調整
    def _adjustSpeedForCooperation(self):
        if self.receiving_cooperative_from_id:
            supporting_vehicle = vehicle_instances[self.receiving_cooperative_from_id]
            supporting_vehicle_speed = supporting_vehicle.speed
            own_position = traci.vehicle.getLanePosition(self.id)

            # 車線変更に適した速度に調整
            # TODO 必要であれば適切に車線変更側の速度も調整する
            target_speed = supporting_vehicle_speed
            safe_duration = self._calculateSafeDecelDuration(self.speed - target_speed)
            traci.vehicle.slowDown(self.id, target_speed, safe_duration)

            # 協調車両の速度も調整
            supporting_vehicle._adjustSupportingSpeed(self.speed, own_position)

    # 車線変更を支援する側の速度の調整
    def _adjustSupportingSpeed(self, requesting_speed, requesting_position):
        own_position = traci.vehicle.getLanePosition(self.id)
        # 安全な車間距離を確保しつつ速度を調整
        target_speed = self._calculateSupportingSpeed(
            requesting_speed, requesting_position, own_position
        )
        safe_duration = self._calculateSafeDecelDuration(self.speed - target_speed)
        traci.vehicle.slowDown(self.id, target_speed, safe_duration)

    # 車線変更を支援する側の適切な速度を計算
    def _calculateSupportingSpeed(self, requesting_speed, requesting_pos, own_pos):
        position_diff = requesting_pos - own_pos

        if position_diff < self.safety_gap:
            # 車間距離が不足 → より大きく減速して車間を開ける
            deceleration_rate = max(0.6, position_diff / self.safety_gap)
            return max(requesting_speed * deceleration_rate, 0)
        else:
            # 適切な車間距離がある → 緩やかに減速して車間を維持
            return requesting_speed * 0.9

    # 協調車両を決定
    def _decideYieldingVehicle(self):
        if self.action == "change_left":
            candidates = self.left_followers
        elif self.action == "change_right":
            candidates = self.right_followers
        else:
            return

        if not candidates:
            return

        # 優先順位に基づいて候補を選定
        viable_candidates = []

        for vehicle_id, distance in candidates:
            if vehicle_id in vehicle_instances:
                vehicle = vehicle_instances[vehicle_id]
                if (
                    vehicle.priority > self.priority and vehicle.is_yielding == False
                ):  # 自身より優先度が低い車輌に限定
                    viable_candidates.append((vehicle_id, distance))

        # 候補車両があれば、最も近い車両を選択
        if viable_candidates:
            # distanceで並び替え済みなので最初の要素を選択
            self.receiving_cooperative_from_id = viable_candidates[0][0]

    def _resetLaneChangeState(self):
        self.is_lane_changing = False
        self.action = "stay"
        self.receiving_cooperative_from_id = None

        if self.receiving_cooperative_from_id in vehicle_instances:
            supporting_vehicle = vehicle_instances[self.receiving_cooperative_from_id]
            supporting_vehicle.is_yielding = False
            supporting_vehicle.providing_cooperative_to_id = None

    # 自身の行動（priority） を決定
    def decideNextActionAndPriority(self):
        # 無効な道路上の場合は何もしない
        if self.road is None or self.road.startswith(":"):
            self.action = "stay"
            self.priority = 6

        # 車線変更中の場合は行動を継続
        if self.action == "change_left" or self.action == "change_right":
            return

        # 現在のレーンと経路に基づくルールを取得
        key = (self.lane, self.route)
        rule = self.lane_change_rules.get(key)

        if not rule:
            self.priority = 6

        # 条件を満たすか確認
        if all(condition() for condition in rule["conditions"]):
            self.action = rule["action"]
            self.priority = rule["priority"]
            self.is_lane_changing = True
        else:
            self.action = "stay"
            self.priority = 5

    # 車線変更が安全かどうか
    # TODO 車線変更操作の安全性を判断する関数を改善する

    # def _canChangeLane(self, direction):
    #     # 隣接レーンの車両を取得
    #     followers = self.left_followers if direction == "left" else self.right_followers
    #     leaders = self.left_leaders if direction == "left" else self.right_leaders

    #     # 後続車両との安全性チェック
    #     if followers:
    #         follower_id, follower_distance = followers[0]  # 最も近い後続車両
    #         if follower_id in vehicle_instances:
    #             follower = vehicle_instances[follower_id]

    #             # 後続車両のsafety_gapを使用
    #             if follower_distance < follower.safety_gap:
    #                 return False

    #     # 先行車両との安全性チェック
    #     if leaders:
    #         leader_id, leader_distance = leaders[0]  # 最も近い先行車両
    #         if leader_id in vehicle_instances:
    #             leader = vehicle_instances[leader_id]
    #             # 自車両のsafety_gapを使用
    #             if leader_distance < leader.safety_gap:
    #                 return False

    #     return True
    def _canChangeLane(self, direction):
        """
        車線変更の安全性を判断する
        最大加減速度を考慮して必要な車間距離を動的に計算
        """
        followers = self.left_followers if direction == "left" else self.right_followers
        leaders = self.left_leaders if direction == "left" else self.right_leaders

        # 後続車両との安全性チェック
        if followers:
            follower_id, follower_distance = followers[0]
            if follower_id in vehicle_instances:
                follower = vehicle_instances[follower_id]
                speed_diff = follower.speed - self.speed

                if speed_diff <= 0:  # 後続車両が遅い場合
                    # 最小限の車間距離のみ要求
                    required_distance = self.length + minGap
                else:
                    # 後続車両の制動可能距離を計算
                    # v^2 = v0^2 + 2ax から制動距離を計算
                    follower_stopping_distance = (speed_diff**2) / (2 * abs(maxDecel))
                    # 反応時間分の走行距離
                    reaction_distance = speed_diff * reactionTime
                    # 車線変更時は通常のsafety_gapより短い距離を許容
                    required_distance = (
                        self.length
                        + minGap
                        + follower_stopping_distance * 0.7  # 制動距離を70%に緩和
                        + reaction_distance * 0.8
                    )  # 反応距離を80%に緩和

                if follower_distance < required_distance:
                    return False

        # 先行車両との安全性チェック
        if leaders:
            leader_id, leader_distance = leaders[0]
            if leader_id in vehicle_instances:
                leader = vehicle_instances[leader_id]
                speed_diff = self.speed - leader.speed

                if speed_diff <= 0:  # 自車両が遅い場合
                    # 最小限の車間距離のみ要求
                    required_distance = self.length + minGap
                else:
                    # 自車両の制動可能距離を計算
                    own_stopping_distance = (speed_diff**2) / (2 * abs(maxDecel))
                    # 反応時間分の走行距離
                    reaction_distance = speed_diff * reactionTime
                    # 車線変更時は通常のsafety_gapより短い距離を許容
                    required_distance = (
                        self.length
                        + minGap
                        + own_stopping_distance * 0.7  # 制動距離を70%に緩和
                        + reaction_distance * 0.8
                    )  # 反応距離を80%に緩和

                if leader_distance < required_distance:
                    return False

        return True

    # 車線変更を実行した際に速度が上昇するか
    def _isPredictedSpeedIncrease(self, direction):
        current_lane_id = f"{self.road}_{self.lane}"
        target_lane_num = self.lane + (1 if direction == "left" else -1)
        target_lane_id = f"{self.road}_{target_lane_num}"

        current_lane_avg_speed = traci.lane.getLastStepMeanSpeed(current_lane_id)
        target_lane_avg_speed = traci.lane.getLastStepMeanSpeed(target_lane_id)

        return target_lane_avg_speed > current_lane_avg_speed * (
            1 + SPEED_IMPROVEMENT_THRESHOLD / 100
        )
