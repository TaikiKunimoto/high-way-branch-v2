import math
import os
import sys

from status.status import CarAction, CarStatus, LaneChangeStatus

if "SUMO_HOME" in os.environ:
    tools = os.path.join(os.environ["SUMO_HOME"], "tools")
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")
import traci  # noqa

maxSpeed = 27  # [m/s]
maxAccel = 5.0  # [m/ss]
maxDecel = -5.0  # [m/ss]
minGap = 2.8  # [m]
reactionTime = 0.75  # [s]
frictionCoefficient = 0.7  # 摩擦係数
LANE_WIDTH = 3.2  # [m]
LANE_CHANGE_MARGIN = 400.0  # [m] 渋滞発生地点の何メートル手前から車線変更を許可するか
SPEED_IMPROVEMENT_THRESHOLD = 40.0  # 車線変更による速度改善の閾値 [%]
MAINLANE_LENGTH = 1500  # [m]

timeStep = 0.1  # [s]

vehicle_instances = {}  # グローバルな車輌管理辞書


class SimpleCAV:
    # constructor
    def __init__(self, vehID):
        self.id = str(vehID)
        vehicle_instances[self.id] = self

        # sumoによる車線変更を無効化
        traci.vehicle.setLaneChangeMode(vehID=self.id, laneChangeMode=0)
        # control vehicle speed by traci
        traci.vehicle.setSpeedMode(vehID=self.id, speedMode=0)
        traci.vehicle.setMinGap(self.id, 2.8)  # default 2.5
        traci.vehicle.setTau(self.id, 1.0)  # default 1.0

        self.typeID = traci.vehicle.getTypeID(self.id)
        self.route = traci.vehicle.getRouteID(self.id)
        self.road = None
        self.laneID = traci.vehicle.getLaneID(self.id)
        self.lane = None
        self.lane_change_status = LaneChangeStatus.SPEED_IMPROVEMENT_ONLY
        self.status = CarStatus.NORMAL
        self.action = CarAction.STAY
        self.priority = 0  # 0(normal), 1(emergency)
        self.last_lane_change_time = None  # Sumo Time
        self.lane_change_pending = False
        self.receiving_cooperative_from_id = None  # 協調中に譲ってもらう車両のID
        self.providing_cooperative_to_id = None  # 協調して譲る車両のID
        self.leader_distance = None  # 前方車両との距離
        self.leader_speed = None  # 前方車両の速度
        self.current_lane_leaders = None  # 現在のレーンの前方車両
        self.current_lane_followers = None
        self.left_followers = None  # 左後続車両
        self.right_followers = None  # 右後続車両
        self.left_leaders = None  # 左前方車両
        self.right_leaders = None  # 右前方車両

        self.lane_pos = None
        self.pos_x = None
        self.pos_y = None
        self.angle = None
        self.speed = 0
        self.accel = 0
        self.leader = None
        self.length = 5.0
        self.width = 1.8
        self.reaction_distance = 0  # 空走距離 [m]
        self.breaking_distance = 0  # 制動距離 [m]
        self.safety_gap = self.reaction_distance + self.breaking_distance + minGap
        self.do_not_speed_up = False

        # [m] 車線変更の目的車線にいる前方車両との安全距離
        self.required_distance_from_leader = None
        self.current_distance_from_leader = None
        # [m] 車線変更の目的車線にいる後方車両との安全距離
        self.required_distance_from_follower = None
        self.current_distance_from_follower = None

        self.lane_change_leader_speed = None

        self.simTime = traci.simulation.getTime()

        self.isWaitAgree = False

        self.collision_counter = 0
        self.caution_counter = 0
        self.emergency_brake_counter = 0

        self.departure_time = None
        self.arrival_time = None

        self.speed_history = []

        """ 車線変更ルールテーブルの初期化 """
        self.lane_change_rules = {
            # Lane 2のルール
            (2, "speed"): {
                "action": CarAction.CHANGE_RIGHT,
                "priority": 0,
                "conditions": [
                    lambda: self.lane_change_status
                    == LaneChangeStatus.SPEED_IMPROVEMENT_ONLY,
                    lambda: self._isPredictedSpeedIncrease("right"),
                ],
            },
            (2, "r_exit"): {"action": CarAction.STAY, "priority": 0, "conditions": []},
            (2, "r_pass"): {
                "action": CarAction.CHANGE_RIGHT,
                "priority": 0,
                "conditions": [
                    lambda: self.lane_change_status
                    == LaneChangeStatus.SPEED_IMPROVEMENT_ONLY
                    or self.lane_change_status == LaneChangeStatus.ALL_ALLOWED,
                    lambda: self._isPredictedSpeedIncrease("right"),
                ],
            },
            # Lane 1のルール
            (1, "speed_left"): {
                "action": CarAction.CHANGE_LEFT,
                "priority": 0,
                "conditions": [
                    lambda: self.lane_change_status
                    == LaneChangeStatus.SPEED_IMPROVEMENT_ONLY,
                    lambda: self._isPredictedSpeedIncrease("left"),
                ],
            },
            (1, "speed_right"): {
                "action": CarAction.CHANGE_RIGHT,
                "priority": 0,
                "conditions": [
                    lambda: self.lane_change_status
                    == LaneChangeStatus.SPEED_IMPROVEMENT_ONLY,
                    lambda: self._isPredictedSpeedIncrease("right"),
                ],
            },
            (1, "r_exit"): {
                "action": CarAction.CHANGE_LEFT,
                "priority": 0,
                "conditions": [
                    lambda: self.lane_change_status == LaneChangeStatus.ALL_ALLOWED,
                ],
            },
            (1, "r_pass_left"): {
                "action": CarAction.CHANGE_LEFT,
                "priority": 0,
                "conditions": [
                    lambda: self.lane_change_status == LaneChangeStatus.ALL_ALLOWED,
                    lambda: self._isPredictedSpeedIncrease("left"),
                ],
            },
            (1, "r_pass_right"): {
                "action": CarAction.CHANGE_RIGHT,
                "priority": 0,
                "conditions": [
                    lambda: self.lane_change_status == LaneChangeStatus.ALL_ALLOWED,
                    lambda: self._isPredictedSpeedIncrease("right"),
                ],
            },
            # Lane 0のルール
            (0, "speed"): {
                "action": CarAction.CHANGE_LEFT,
                "priority": 0,
                "conditions": [
                    lambda: self.lane_change_status
                    == LaneChangeStatus.SPEED_IMPROVEMENT_ONLY,
                    lambda: self._isPredictedSpeedIncrease("left"),
                ],
            },
            (0, "r_exit"): {
                "action": CarAction.CHANGE_LEFT,
                "priority": 0,
                "conditions": [
                    lambda: self.lane_change_status == LaneChangeStatus.ALL_ALLOWED,
                ],
            },
            (0, "r_pass"): {
                "action": CarAction.CHANGE_LEFT,
                "priority": 0,
                "conditions": [
                    lambda: self.lane_change_status
                    == LaneChangeStatus.SPEED_IMPROVEMENT_ONLY
                    or self.lane_change_status == LaneChangeStatus.ALL_ALLOWED,
                    lambda: self._isPredictedSpeedIncrease("left"),
                ],
            },
        }

    """ 車輌の実際の出発時刻を取得 """

    def get_departure_time(self):
        self.departure_time = traci.vehicle.getDeparture(self.id)

    """ 車輌の実際の到着時刻を取得 """

    def get_arrival_time(self):
        self.arrival_time = traci.simulation.getTime()

    """ 自身のステータスを更新 """

    def updateStatus(self):
        if self.status == CarStatus.LANE_CHANGED:
            self._resetLaneChangeState()

        self.simTime = traci.simulation.getTime()
        self.speed_history.append(traci.vehicle.getSpeed(self.id))

        # update own position
        pos = traci.vehicle.getPosition(self.id)
        self.pos_x = pos[0]
        self.pos_y = pos[1]
        self.lane_pos = traci.vehicle.getLanePosition(self.id)

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

        # 分流車両の車線変更が間に合わない場合優先度を最大にする
        # 分岐地点の50m手前で車線変更できていない場合は優先度を最大にする
        if self.road == "MainLane1" and self.lane_pos > MAINLANE_LENGTH - 50:
            if self.route == "r_exit" and self.lane != 2:
                self.priority = 1

        if self.road != "MainLane1" and self.status != CarStatus.NORMAL:
            self._resetLaneChangeState()

        # 車線変更が可能なポイントを通過したら車線変更を可能にする
        if self.lane_change_status == LaneChangeStatus.SPEED_IMPROVEMENT_ONLY:
            if self._hasPassedLaneChangePoint():
                self.lane_change_status = LaneChangeStatus.ALL_ALLOWED
                # 協調車線変更が可能になったタイミングで行動を初期化, 協調中であればそのステータスは維持
                self._resetLaneChangeStateKeepYielding()

        if self.lane_change_status == LaneChangeStatus.ALL_ALLOWED:
            if self.road != "MainLane1" or self.lane_pos >= MAINLANE_LENGTH - (
                self.length + minGap
            ):
                # 車線変更は禁止するが協調中のステータスは維持, 優先度が1の場合は車線変更可能
                self.lane_change_status = LaneChangeStatus.UNAVAILABLE
                self._resetLaneChangeStateKeepYielding()

        self._getFollowerAndLeader()

        # 車線変更が可能かを判断するリストをリセット
        self.current_distance_from_follower = None
        self.required_distance_from_follower = None
        self.current_distance_from_leader = None
        self.required_distance_from_leader = None
        self.lane_change_leader_speed = None
        self.do_not_speed_up = False

        # debug 協調車両同士が正しく設定されているか確認
        # TODO この関数入れるだけでめっちゃ重くなるから消したい
        if self.providing_cooperative_to_id is not None:
            if self.providing_cooperative_to_id in vehicle_instances:
                supporting_vehicle = vehicle_instances[self.providing_cooperative_to_id]

                if self.lane == supporting_vehicle.lane:
                    print(
                        f"Error: {self.id} is providing cooperation to {self.providing_cooperative_to_id} but they are in the same lane"
                    )
                    supporting_vehicle._resetLaneChangeState()

                if supporting_vehicle.action == CarAction.CHANGE_LEFT:
                    if supporting_vehicle.lane != self.lane - 1:
                        print(
                            f"Error: {self.id} is providing cooperation to {self.providing_cooperative_to_id} but the target lane is not correct"
                        )
                        supporting_vehicle._resetLaneChangeState()
                elif supporting_vehicle.action == CarAction.CHANGE_RIGHT:
                    if supporting_vehicle.lane != self.lane + 1:
                        print(
                            f"Error: {self.id} is providing cooperation to {self.providing_cooperative_to_id} but the target lane is not correct"
                        )
                        supporting_vehicle._resetLaneChangeState()

                if self.id != supporting_vehicle.receiving_cooperative_from_id:
                    print(
                        f"Error: {self.id} is providing cooperation to {self.providing_cooperative_to_id} but receiving cooperation from {supporting_vehicle.receiving_cooperative_from_id}"
                    )

    """ 自身の行動（priority） を決定 """

    def decideNextActionAndPriority(self):
        # 無効な道路上の場合は何もしない
        if self.road != "MainLane1":
            self.action = CarAction.STAY
            self.priority = 0

        # 車線変更中の場合は行動を継続
        if self.lane_change_status == LaneChangeStatus.ALL_ALLOWED and (
            self.status == CarStatus.LANE_CHANGING
            or self.status == CarStatus.YIELDING
            or self.status == CarStatus.LANE_CHANGED
        ):
            return

        # 現在のレーンと経路に基づくルールを取得
        if self.lane != 1:
            base_key = (self.lane, self.route)
            speed_key = (self.lane, "speed")
        else:
            if self.lane_change_status == LaneChangeStatus.SPEED_IMPROVEMENT_ONLY:
                base_key = (self.lane, self.route)
                if self._isPredictedSpeedIncrease("left"):
                    speed_key = (self.lane, "speed_left")
                elif self._isPredictedSpeedIncrease("right"):
                    speed_key = (self.lane, "speed_right")
                else:
                    speed_key = None
            else:
                speed_key = None
                if self.route == "r_exit":
                    base_key = (self.lane, self.route)
                else:
                    if self._isPredictedSpeedIncrease("left"):
                        base_key = (self.lane, self.route + "_left")
                    elif self._isPredictedSpeedIncrease("right"):
                        base_key = (self.lane, self.route + "_right")
                    else:
                        base_key = (self.lane, self.route)

        if self.lane_change_status == LaneChangeStatus.SPEED_IMPROVEMENT_ONLY:
            rule = self.lane_change_rules.get(speed_key)
        else:
            rule = self.lane_change_rules.get(base_key)

        if not rule:
            self.action = CarAction.STAY
            self.priority = 0
            return

        # 条件を満たすか確認
        if all(condition() for condition in rule["conditions"]):
            self.action = rule["action"]
            self.priority = rule["priority"]
        else:
            self.action = CarAction.STAY
            self.priority = 0

        if self.action != CarAction.STAY:
            self.status = CarStatus.LANE_CHANGING

    """ 車両の速度を調整 """

    def controlSpeed(self):
        if self.leader_distance is not None and self.leader_distance < minGap:
            self._emergencyBreak(self.leader_speed)
            return

        # 協調フェーズの場合は加速は行わない
        if (
            self.status == CarStatus.YIELDING
            or self.status == CarStatus.LANE_CHANGING
            or self.status == CarStatus.LANE_CHANGED
        ):
            self.do_not_speed_up = True
        else:
            self.do_not_speed_up = False

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
                self.leader_distance, speed_diff
            )
            # 前方車両との距離 > safety_gap の場合
            if self.leader_distance >= self.safety_gap:
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

            # 前方車両との距離 < safety_gap の場合
            else:
                if self.do_not_speed_up:
                    return
                if speed_diff >= 0:
                    # 通常の減速
                    if self.leader_speed > 1:
                        target_speed = self.leader_speed - 1
                    else:
                        target_speed = 0
                    duration = min(ttc_with_safety_margin, min_duration)
                    traci.vehicle.slowDown(self.id, target_speed, duration)
                else:
                    return

    """ 車線変更を実行 """

    def executeLaneChange(self):
        if (
            self.lane_change_status == LaneChangeStatus.UNAVAILABLE
            and self.priority != 1
        ):
            return

        if self.action == CarAction.STAY:
            return

        # 車線変更開始地点以前の車線変更は協調しない
        if self.lane_change_status == LaneChangeStatus.SPEED_IMPROVEMENT_ONLY:
            cooperation_mode = False
        elif self.lane_change_status == LaneChangeStatus.ALL_ALLOWED:
            cooperation_mode = True

        if self.last_lane_change_time is not None:
            if self.simTime - self.last_lane_change_time < 7:
                self.lane_change_pending = True
            else:
                self.lane_change_pending = False
        else:
            self.lane_change_pending = False

        direction = "left" if self.action == CarAction.CHANGE_LEFT else "right"
        lane_change_amount = 1 if direction == "left" else -1

        if self.lane_change_pending:
            # 車線変更がpendingの最中は協調車両を選ばない
            self._adjustSpeedForCooperation()
            return
        elif self._canChangeLane(direction):
            # 意図しない挙動でシミュレーションが止まるのを防ぐ 衝突は消滅したけど一応残した
            if self.road != "MainLane1":
                self._resetLaneChangeState()
                return

            self.last_lane_change_time = self.simTime

            # 車線変更が可能な場合は実行
            traci.vehicle.changeLane(self.id, self.lane + lane_change_amount, 0)
            self.status = CarStatus.LANE_CHANGED
        else:
            if cooperation_mode:
                if self.receiving_cooperative_from_id in vehicle_instances:
                    supporting_vehicle = vehicle_instances[
                        self.receiving_cooperative_from_id
                    ]
                    position_diff = self.pos_x - supporting_vehicle.pos_x
                else:
                    position_diff = -1 * math.inf

                # 車線変更ができず、まだ協調車両がいない場合 or 協調車輌が自身より前方にいる場合
                if (
                    self.receiving_cooperative_from_id is None
                    or position_diff <= self.length + minGap
                ):
                    # 新たに協調車両を決定するため過去の情報をリセット
                    if self.receiving_cooperative_from_id is not None:
                        supporting_vehicle = vehicle_instances[
                            self.receiving_cooperative_from_id
                        ]
                        supporting_vehicle.status = CarStatus.NORMAL
                        supporting_vehicle.priority = 0
                        supporting_vehicle.providing_cooperative_to_id = None
                        supporting_vehicle.do_not_speed_up = False
                        self.receiving_cooperative_from_id = None

                    self._decideYieldingVehicle()
                    self._requestCooperation()
                # 協調車輌と自身の速度を調整
                self._adjustSpeedForCooperation()
            else:
                # 協調が許可されていない場合(速度向上車線変更)の場合は自身の速度のみ調整
                self._adjustSpeedForCooperation()

    """ 適切な車間距離の計算 """

    def _calculateSafetyGap(self):
        speed_kmh = self.speed * 3.6
        # 空走距離
        self.reaction_distance = self.speed * reactionTime
        # 制動距離
        self.breaking_distance = (speed_kmh**2) / (254.016 * frictionCoefficient)
        # 安全距離
        self.safety_gap = self.reaction_distance + self.breaking_distance + minGap

    """ 後続車輌と先行車輌を取得する """

    def _getFollowerAndLeader(self):
        self._resetFollowerAndLeaderVehicles()

        if self.road is None or self.lane is None or self.road != "MainLane1":
            return

        own_position = self.lane_pos

        # レーン番号に基づいて確認すべき隣接レーンを決定
        check_lanes = [("current", self.lane)]
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
            elif direction == "current":
                self.current_lane_followers = followers
                self.current_lane_leaders = leaders

    def _resetFollowerAndLeaderVehicles(self):
        self.current_lane_followers = None
        self.current_lane_leaders = None
        self.left_followers = None
        self.left_leaders = None
        self.right_followers = None
        self.right_leaders = None

    """ 車線変更が可能なポイントを通過したかどうか """

    def _hasPassedLaneChangePoint(self):
        current_pos = self.lane_pos
        merge_start_pos = MAINLANE_LENGTH - LANE_CHANGE_MARGIN

        if current_pos > merge_start_pos:
            return True
        return False

    """ 制限速度に基づいて速度を調整 """

    def _controlSpeedBySpeedLimit(self, speed_limit):
        # 減速
        if self.speed > speed_limit:
            safe_duration = self._calculateSafeDecelDuration(self.speed - speed_limit)
            traci.vehicle.slowDown(self.id, speed_limit, safe_duration)
            return
        # 加速
        else:
            if self.do_not_speed_up:
                return
            safe_duration = self._calculateSafeAccelDuration(speed_limit - self.speed)
            traci.vehicle.slowDown(self.id, speed_limit, safe_duration)
            return

    """ 最大減速で速度差を0にするために必要な時間を計算 """

    def _calculateSafeDecelDuration(self, speed_diff):
        if speed_diff <= 0:
            return 0
        return speed_diff / abs(maxDecel)

    """ 最大加速で速度差を0にするために必要な時間を計算 """

    def _calculateSafeAccelDuration(self, speed_diff):
        if speed_diff <= 0:
            return 0
        return speed_diff / abs(maxAccel)

    """ 現在の速度差で進んだ際に衝突までにかかる時間(TTC) """

    def _calculateTTC(self, distance, speed_diff):
        if speed_diff <= 0:
            return math.inf
        return distance / speed_diff

    """ 衝突回避のための速度調整 """

    def _emergencyBreak(self, targetSpeed):
        targetSpeed = min(targetSpeed, self.speed, 1)
        self.emergency_brake_counter += 1
        traci.vehicle.setSpeed(self.id, targetSpeed)

    """ 協調車輌に協調を要求 """

    def _requestCooperation(self):
        if self.receiving_cooperative_from_id in vehicle_instances:
            supporting_vehicle = vehicle_instances[self.receiving_cooperative_from_id]
            supporting_vehicle._resetLaneChangeState()
            supporting_vehicle.status = CarStatus.YIELDING
            supporting_vehicle.providing_cooperative_to_id = self.id

    """ 協調車輌と自身の速度を調整 """

    def _adjustSpeedForCooperation(self):
        if self.leader_distance is not None and self.leader_distance < minGap:
            self._emergencyBreak(self.leader_speed)
            return

        # 自身の速度を車線変更に適した速度に調整
        # 目的車線の前方に車線変更を妨げる車両がいるなら,その車両の速度を参考にする
        if self.lane_change_leader_speed is not None:
            target_speed = self._calculateSupportingSpeed(
                self.lane_change_leader_speed,
                self.current_distance_from_leader,
                self.required_distance_from_leader,
            )
            # 後方車輌が徐々に減速しているのでsafe_durationがstepごとに大きくなってしまい減速が遅くなるため明示的に指定してる
            traci.vehicle.slowDown(self.id, target_speed, 0.5)
        # 目的車線の前方に車線変更を妨げる車両がいないなら現在の車線のリーダーの速度を参考にする
        elif self.leader_speed is not None:
            target_speed = self.leader_speed
            traci.vehicle.slowDown(self.id, target_speed, 0.5)
        # 前方に車両がいないなら制限速度に合わせる
        else:
            # 現在のレーンと制限速度を取得
            current_lane = f"{self.road}_{self.lane}"
            speed_limit = traci.lane.getMaxSpeed(current_lane)
            self._controlSpeedBySpeedLimit(speed_limit)

        # 協調車両がいる場合は相手の速度も調整
        if self.receiving_cooperative_from_id:
            supporting_vehicle = vehicle_instances[self.receiving_cooperative_from_id]
            own_position = self.lane_pos

            supporting_vehicle._adjustSupportingSpeed(
                self.speed,
                own_position,
                self.current_distance_from_follower,
                self.required_distance_from_follower,
            )

    """ 車線変更を支援する側の速度の調整 """

    def _adjustSupportingSpeed(
        self, requesting_speed, requesting_position, current_distance, required_distance
    ):
        if self.leader_distance is not None and self.leader_distance < minGap:
            self._emergencyBreak(self.leader_speed)
            return

        # 安全な車間距離を確保しつつ速度を調整
        target_speed = self._calculateSupportingSpeed(
            requesting_speed, current_distance, required_distance
        )
        safe_duration = self._calculateSafeDecelDuration(self.speed - target_speed)
        traci.vehicle.slowDown(self.id, target_speed, safe_duration)

    """ 車線変更を支援する側の適切な速度を計算 """

    def _calculateSupportingSpeed(
        self, requesting_speed, current_distance, required_distance
    ):
        if current_distance is None or required_distance is None:
            return requesting_speed

        position_diff = required_distance - current_distance

        # 車間距離が不足 → より大きく減速して車間を開ける
        deceleration_rate = position_diff / required_distance
        return requesting_speed * deceleration_rate

    """ 協調車両を決定 """

    def _decideYieldingVehicle(self):
        if self.action == CarAction.CHANGE_LEFT:
            candidates = self.left_followers
        elif self.action == CarAction.CHANGE_RIGHT:
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
                    vehicle.status == CarStatus.NORMAL
                    or vehicle.status == CarStatus.LANE_CHANGING
                ):
                    viable_candidates.append((vehicle_id, distance))

        # 候補車両があれば、最も近い車両を選択
        if viable_candidates:
            if self.speed == 0:
                # 速度が0の場合は最も近い車両の次に近い車両を選択
                self.receiving_cooperative_from_id = (
                    viable_candidates[1][0] if len(viable_candidates) > 1 else None
                )
            # 速度が0でない場合は最も近い車両を選択
            self.receiving_cooperative_from_id = viable_candidates[0][0]

    def _resetLaneChangeState(self):
        self.status = CarStatus.NORMAL
        self.action = CarAction.STAY
        self.priority = 0
        self.required_distance_from_follower = None
        self.current_distance_from_follower = None
        self.required_distance_from_leader = None
        self.current_distance_from_leader = None
        self.lane_change_leader_speed = None
        self.do_not_speed_up = False
        self.lane_change_pending = False

        if self.receiving_cooperative_from_id in vehicle_instances:
            supporting_vehicle = vehicle_instances[self.receiving_cooperative_from_id]
            supporting_vehicle.status = CarStatus.NORMAL
            supporting_vehicle.priority = 0
            supporting_vehicle.providing_cooperative_to_id = None
            supporting_vehicle.do_not_speed_up = False
            self.receiving_cooperative_from_id = None

    def _resetLaneChangeStateKeepYielding(self):
        if self.status != CarStatus.YIELDING:
            self.status = CarStatus.NORMAL
        self.action = CarAction.STAY
        self.priority = 0
        self.required_distance_from_follower = None
        self.current_distance_from_follower = None
        self.required_distance_from_leader = None
        self.current_distance_from_leader = None
        self.lane_change_leader_speed = None
        self.do_not_speed_up = False
        self.lane_change_pending = False

        if self.receiving_cooperative_from_id in vehicle_instances:
            supporting_vehicle = vehicle_instances[self.receiving_cooperative_from_id]
            supporting_vehicle.status = CarStatus.NORMAL
            supporting_vehicle.priority = 0
            supporting_vehicle.providing_cooperative_to_id = None
            supporting_vehicle.do_not_speed_up = False
            self.receiving_cooperative_from_id = None

    """ 車線変更が安全かどうか """
    """ 最大加減速度、車間距離、一つ挟んだ車線とのコリジョンを考慮 """

    def _canChangeLane(self, direction):
        target_lane = self.lane + (1 if direction == "left" else -1)
        if target_lane == 1 and self.road == "MainLane1":
            own_pos = self.lane_pos
            check_range = self.safety_gap

            opposite_lane = 2 if self.lane == 0 else 0
            opposite_lane_vehicle_ids = traci.lane.getLastStepVehicleIDs(
                f"{self.road}_{opposite_lane}"
            )

            for veh_id in opposite_lane_vehicle_ids:
                if veh_id in vehicle_instances:
                    opposite_vehicle = vehicle_instances[veh_id]
                    veh_pos = traci.vehicle.getLanePosition(veh_id)

                    # 一つ挟んだ車線とのコリジョンが発生する場合には車線変更を許可しない
                    if (
                        abs(veh_pos - own_pos) < check_range
                        and opposite_vehicle.action != CarAction.STAY
                        and own_pos < veh_pos
                    ):
                        return False

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
                    required_distance = self.length + minGap * 1.5
                else:
                    required_distance = (
                        self.length
                        + minGap * 1.5
                        + follower.safety_gap * (speed_diff / maxSpeed)
                    )
                    # 必要な後続との距離は、車両長 + minGap + 後続の制動距離(+速度差)を考慮した値
                    # 速度差が大きい場合には制動距離をより考慮したい。速度差は 0 ~ 27のレンジなので、それを0 ~ 1に正規化して考慮する

                if follower_distance < required_distance:
                    self.current_distance_from_follower = follower_distance
                    self.required_distance_from_follower = required_distance

        # 先行車両との安全性チェック
        if leaders:
            leader_id, leader_distance = leaders[0]
            if leader_id in vehicle_instances:
                leader = vehicle_instances[leader_id]
                speed_diff = self.speed - leader.speed

                if speed_diff <= 0:  # 自車両が遅い場合
                    # 最小限の車間距離のみ要求
                    required_distance = self.length + minGap * 1.5
                else:
                    # 車線変更時は通常のsafety_gapより短い距離を許容
                    required_distance = (
                        self.length
                        + minGap * 1.5
                        + self.safety_gap * (speed_diff / maxSpeed)
                    )

                if leader_distance < required_distance:
                    self.current_distance_from_leader = leader_distance
                    self.required_distance_from_leader = required_distance
                    self.lane_change_leader_speed = leader.speed

        if (
            self.required_distance_from_follower is not None
            or self.required_distance_from_leader is not None
        ):
            return False

        return True

    """ 車線変更を実行した際に速度が上昇するか """

    def _isPredictedSpeedIncrease(self, direction):
        if direction == "left":
            target_lane_leaders = self.left_leaders
        elif direction == "right":
            target_lane_leaders = self.right_leaders
        else:
            return False

        if not target_lane_leaders:
            return True

        current_lane_leader_speeds = [
            traci.vehicle.getSpeed(leader[0]) for leader in self.current_lane_leaders
        ]
        if len(current_lane_leader_speeds) > 0:
            current_lane_avg_leader_speed = sum(current_lane_leader_speeds) / len(
                current_lane_leader_speeds
            )
        else:
            current_lane_avg_leader_speed = 0
        # 自分自身の速度と走行中の車線の平均速度の大きい方を取得
        baseline_speed = max(self.speed, current_lane_avg_leader_speed)

        target_lane_leader_speeds = [
            traci.vehicle.getSpeed(leader[0]) for leader in target_lane_leaders
        ]
        target_lane_avg_leader_speed = sum(target_lane_leader_speeds) / len(
            target_lane_leader_speeds
        )

        return target_lane_avg_leader_speed > baseline_speed * (
            1 + SPEED_IMPROVEMENT_THRESHOLD / 100
        )
