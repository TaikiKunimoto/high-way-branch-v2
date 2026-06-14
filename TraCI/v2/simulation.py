"""v2 のシミュレーション（状態保持＋実行）と毎step メインループ（2フェーズの骨格）。

``V2Simulation`` が1回のシミュレーションの状態保持と実行（run）・流入・到着処理・統計配線を担う。
調停（Layer1=RSU/EDF）・実行（Layer2）・観測（V2CAV）・障害物（Obstacle）は各クラスのメソッドに委譲する。
``custom.py`` の run() を踏襲しつつ V2CAV を用い、タイムスペース図(matplotlib)出力は省略する。
"""

import csv
from datetime import datetime
import os
import random
import sys

from pydantic import BaseModel, Field

from simulationStatistics.simulation_statistics import SimulationStatistics
from utils.traci_wrapper import (
    get_colliding_veh_id_list,
    get_edge_lane_number,
    get_lane_last_step_veh_ids,
    get_sim_arrived_veh_id_list,
    get_sim_departed_veh_id_list,
    get_sim_time,
    get_veh_id_list,
    get_veh_lane_position,
    get_veh_speed,
)
from v2.constants import (
    CONGESTION_SPEED,
    MIN_CONGESTED_VEHICLES,
    TC,
    TIME_STEP,
)
from v2.environment import Environment, Group
from v2.lc_request import LCRequest
from v2.obstacle import Obstacle
from v2.pair_executor import Layer2
from v2.priority import EDF
from v2.rsu import RSU, Assignment
from v2.snapshot import Snapshot
from v2.v2_cav import V2CAV

if "SUMO_HOME" in os.environ:
    tools = os.path.join(os.environ["SUMO_HOME"], "tools")
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")
import traci

OUTPUT_DIR = "simulationStatistics/statistics/v2"


class V2Simulation(BaseModel):
    """1回のシミュレーションの状態保持と実行（run）・流入・到着処理を担う。

    可変フィールドは pydantic の default_factory でインスタンスごとに生成され共有されない。
    heavily mutated なので frozen にしない（fields は V2CAV/Environment 等すべて pydantic ネイティブ）。
    """

    simulation_time: float
    env: Environment
    veh_id: int = 0  # 次に投入する車両へ振る連番ID
    # グループ別の流入時刻（環境のグループ定義順を保持）
    group_depart_times: list[tuple[Group, list[float]]] = Field(default_factory=list)
    inflow_through: int = 0  # CSV 用: 必須LCなし車の流入量
    inflow_mlc: int = 0  # CSV 用: 必須LC車の流入量
    vehicles: list[V2CAV] = Field(default_factory=list)
    total_departed: list[str] = Field(default_factory=list)
    exit_vehicles: list[str] = Field(default_factory=list)
    canceled_vehicles: list[str] = Field(default_factory=list)
    collision_history: list[tuple[float, list[str]]] = Field(default_factory=list)
    # 各車線の待ち行列（流入レーン選択の負荷分散に使う）。レーンは環境により可変なので動的に作る
    lane_queues: dict[str, list[str]] = Field(default_factory=dict)

    def run(
        self,
        total_inflow: float,
        mlc_ratio: float,
        stats: SimulationStatistics,
        seed: str,
        obstacle: Obstacle | None = None,  # 突発障害物（指定レーン・位置・時刻）。None なら障害物なし
    ) -> None:
        self._set_environment(total_inflow, mlc_ratio)
        # 突発障害物。発生後、本線レーン数はエスカレーションの回避先選択に使う
        obstacle_num_lanes = get_edge_lane_number(self.env.mainlane_edge) if obstacle is not None else 0
        obstacle_placed_pos: float | None = None
        obstacle_target_id: str | None = None  # 位置到達トリガで pos 手前から監視中の車（pos 到達で停止＝障害物化）
        if obstacle is not None:
            obstacle.validate_for(self.env.mainlane_edge, obstacle_num_lanes, self.env.mainlane_length)

        last_recorded_second = -1
        tail_position_list: list[tuple[float, float]] = []
        max_tail_position = 0.0

        r_pass_departed: list[str] = []
        r_exit_departed: list[str] = []
        r_pass_exited: list[str] = []
        r_exit_exited: list[str] = []
        r_exit_running: dict[str, float] = {}
        running_list: list[str] = []
        tc_accumulator = 0.0
        last_request_log_sec = -1
        tie_events = 0  # Phase A の鍵に同点が出た Tc ラウンド数（デッドロックフリーなら 0）
        double_assign_events = 0  # 同一提供車が二重割当された Tc ラウンド数（横取り禁止なら 0）
        total_lc = 0  # Layer2 で実行された瞬時LCの総数
        snap: Snapshot | None = None  # 直近 Tc のスナップショット（Layer2 実行で参照）
        assignments: list[Assignment] = []  # 直近 Tc の割当（Layer2 で実行）
        req_by_id: dict[str, LCRequest] = {}  # 直近 Tc の要求（id 引き）

        while self._should_continue():
            traci.simulationStep()
            self._check_collision()

            arrived_list = get_sim_arrived_veh_id_list()  # 直近stepで範囲外に出た（到着した）車両ID
            departed_list = get_sim_departed_veh_id_list()  # 直近stepで投入された（出発した）車両ID
            running_list = get_veh_id_list()  # 現在ネットワーク上を走行中の全車両ID

            # tail position の記録（タイムスペース指標用）
            current_time = get_sim_time()
            current_sec = int(current_time)
            if current_sec != last_recorded_second:
                tail_pos = self.env.mainlane_length - self._get_congestion_point()
                tail_position_list.append((current_time, tail_pos))
                max_tail_position = max(max_tail_position, tail_pos)
                last_recorded_second = current_sec

            # --- 到着/未発進/出発処理 と 観測。全車を先に観測し、スナップショット S_t の一貫性を保つ ---
            poplist: list[int] = []  # このstepで到着し self.vehicles から削除する要素インデックス
            active: list[V2CAV] = []  # このstep走行中で観測・調停・制御の対象となる V2CAV
            for index, veh in enumerate(self.vehicles):
                vid = veh.id

                # シミュレーション範囲を出た車両
                if vid in arrived_list:
                    poplist.append(index)
                    self.exit_vehicles.append(vid)
                    veh.record_arrival_time()
                    veh.accumulate_exit_stats(stats, r_pass_exited, r_exit_exited)
                    continue

                # 混雑で未発進の車両
                if vid not in running_list:
                    if vid not in self.canceled_vehicles:
                        self.canceled_vehicles.append(vid)
                    continue

                # 発進した車両の出発時刻記録
                if vid in departed_list:
                    veh.record_departure_time()
                    self.total_departed.append(vid)
                    if veh.target_lane is None:  # 必須LCなし（through）
                        r_pass_departed.append(vid)
                    else:  # 必須LC車
                        r_exit_departed.append(vid)
                    if vid in self.canceled_vehicles:
                        self.canceled_vehicles.remove(vid)

                veh.update_self_observation()  # 自車両の状態更新
                # TODO: ここ本当に必要か，一回で良いのか？
                veh.update_activation(self.env.mainlane_edge)  # MLC要求が活性化された際に一度だけ更新する
                active.append(veh)
                self._update_lane_queue(vid)

                if veh.leader_distance is not None and veh.leader_speed is not None:
                    stats.calculate_TTC(veh.leader_distance, veh.leader_speed, veh.speed)

            # --- 突発障害物（位置到達トリガ）。appear_time 以降、指定レーンで pos に到達した最初の車を停止＝障害物化 ---
            if obstacle is not None and obstacle_placed_pos is None and current_time >= obstacle.appear_time:
                obstacle_target_id, obstacle_placed_pos = obstacle.place(
                    active, self.env.mainlane_edge, obstacle_target_id
                )
            # 障害物より後方・同一レーンの through 車に必須LC（回避）を動的付与＝エスカレーション（コア機構 §4）
            if obstacle is not None and obstacle_placed_pos is not None:
                obstacle.escalate(active, self.env.mainlane_edge, obstacle_placed_pos, obstacle_num_lanes)

            # --- 毎Tc 2フェーズ調停。Phase A（鍵計算）→ Phase B（割当＋役割付与）。Layer2 実行は制御後に行う ---
            tc_accumulator += TIME_STEP
            if tc_accumulator + 1e-9 >= TC:
                tc_accumulator = 0.0
                snap = Snapshot.capture(active, current_time, self.env.mainlane_edge)
                requests = LCRequest.build_all(snap)
                keyed = EDF.order_requests(requests)  # Phase A: 全要求車の鍵を計算し EDF（dist昇順）にソート
                assignments = RSU.arbitrate(keyed, snap)  # Phase B: 鍵順に提供車を占有印つきで確保
                req_by_id = {r.veh_id: r for _, r in keyed}
                RSU.apply_roles(active, assignments)  # 毎Tc フル再構築（提供車=YIELDING / 要求車=LANE_CHANGING）
                if not RSU.keys_unique(keyed):
                    tie_events += 1
                if not RSU.providers_unique(assignments):
                    double_assign_events += 1
                if current_sec % 50 == 0 and current_sec != last_request_log_sec:
                    RSU.log_assignments(current_time, keyed, assignments)
                    last_request_log_sec = current_sec

            # --- 制御（速度）。traci の速度指令は次 step に反映されるため観測順と独立 ---
            for veh in active:
                veh.control_speed()

            # --- Layer2 実行。制御後に呼び、協調減速の slowDown と changeLane が最後の指令になるようにする ---
            if snap is not None:
                total_lc += Layer2.execute_pairs(assignments, req_by_id, snap, {veh.id: veh for veh in active})

            for i in sorted(poplist, reverse=True):
                self.vehicles.pop(i)

            self._add_vehicle()

        # 障害物指定があったのに最後まで配置できなければ、黙って no-op にせず原因つきで失敗させる
        if obstacle is not None and obstacle_placed_pos is None:
            raise RuntimeError(
                f"障害物を配置できませんでした: 指定レーン {obstacle.lane} で pos {obstacle.pos}m に到達する車両が "
                f"appear_time {obstacle.appear_time}s 以降シミュレーション終了まで現れませんでした。"
                "流入量(inflow)・レーン・位置・時刻の指定を確認してください。"
            )

        # 終了時、残車両の統計を更新
        for veh in self.vehicles:
            if veh.id not in running_list:
                continue
            if veh.target_lane is None:  # 必須LCなし（through）
                stats.calculate_vehicle_average_speed("r_pass", veh.speed_history)
            else:  # 必須LC車
                stats.calculate_vehicle_average_speed("r_exit", veh.speed_history)
                if veh.pos_x is not None:
                    r_exit_running[veh.id] = veh.pos_x

        r_exit_running_list = sorted(r_exit_running, key=lambda v: r_exit_running[v], reverse=True)

        collided: set[str] = set()
        for _, vehicles in self.collision_history:
            collided.update(vehicles)
        canceled_without_collision = [v for v in self.canceled_vehicles if v not in collided]

        self._print_simulation_info(running_list)
        print(f"Phase A: Tc rounds with key ties (should be 0): {tie_events}")
        print(f"Phase B: Tc rounds with double-assigned providers (should be 0): {double_assign_events}")
        print(f"Layer2: total instant lane changes executed: {total_lc}")
        total_collisions, total_involved = self._print_collision_summary()
        self._write_tail_csv(seed, tail_position_list)

        results = {
            "total_generated_vehicle": self.veh_id,
            "total_departed_vehicle": self.total_departed,
            "running_vehicle": running_list,
            "exit_vehicle": self.exit_vehicles,
            "r_pass_departed_vehicle": r_pass_departed,
            "r_exit_departed_vehicle": r_exit_departed,
            "r_pass_exit_vehicle": r_pass_exited,
            "r_exit_exit_vehicle": r_exit_exited,
            "r_exit_running_vehicle": r_exit_running_list,
            "canceled_vehicle": canceled_without_collision,
            "traffic_volume": len(self.total_departed) * (3600 / self.simulation_time),
            "total_collisions": total_collisions,
            "total_vehicles_involved": total_involved,
            "max_tail_position": max_tail_position,
        }
        stats.add_result(self.simulation_time, seed, self.inflow_through, self.inflow_mlc, results)
        traci.close()

    def _set_environment(self, total_inflow: float, mlc_ratio: float) -> None:
        """環境のグループ別流入量（総流入 Q × 必須LC比率 f から展開）に従い、流入時刻を乱数で決定（seed で決定的）。

        グループ定義順に random.sample を呼ぶことで決定性を保つ（分流D では through→exiting＝旧 pass→exit と一致）。
        """
        for group, rate in self.env.group_rates(total_inflow, mlc_ratio):
            k = int((self.simulation_time / 3600) * rate)
            seconds = sorted(random.sample(range(int(self.simulation_time)), k))
            # 1秒以上間隔を確保（同一stepへの偏りを避ける）
            times: list[float] = [round(n, 1) + 0.1 for n in seconds]
            self.group_depart_times.append((group, times))
            if group.target_lane is not None:
                self.inflow_mlc += int(rate)
            else:
                self.inflow_through += int(rate)
            print(f"depart_times[{group.name}]:", times)

    def _get_depart_lane(self, edge_id: str, allowed_lanes: tuple[int, ...] | None) -> str:
        """グループの投入レーン候補（None=全レーン）の中で待ち行列が最短のレーンを選ぶ（負荷分散）。"""
        lanes_total: int = get_edge_lane_number(edge_id)
        candidates = [str(i) for i in (allowed_lanes if allowed_lanes is not None else range(lanes_total))]
        queue_length = {lane: len(self.lane_queues.get(lane, [])) for lane in candidates}
        lanes_without_queue = [lane for lane in candidates if queue_length[lane] == 0]
        if lanes_without_queue:
            return random.choice(lanes_without_queue)
        min_length = min(queue_length.values())
        min_lanes = [lane for lane, length in queue_length.items() if length == min_length]
        return random.choice(min_lanes)

    def _add_vehicle(self) -> None:
        """流入時刻に到達した車両を SUMO に追加し V2CAV を生成する。各車は環境のグループから必須LC仕様を受け取る。"""
        sumo_time = get_sim_time()
        for group, depart_times in self.group_depart_times:
            if sumo_time not in depart_times:
                continue
            depart_edge = group.depart_edge if group.depart_edge is not None else self.env.mainlane_edge
            depart_lane = self._get_depart_lane(depart_edge, group.depart_lanes)
            traci.vehicle.add(
                vehID=str(self.veh_id),
                routeID=group.route,
                typeID="CAV",
                departLane=depart_lane,
                departPos="base",
                departSpeed="last",
            )
            self.vehicles.append(
                V2CAV(id=str(self.veh_id), target_lane=group.target_lane, deadline_pos=group.deadline_pos)
            )
            self.lane_queues.setdefault(depart_lane, []).append(str(self.veh_id))
            self.veh_id += 1

    def _update_lane_queue(self, veh_id: str) -> None:
        """走行を開始した車両を待ち行列から外す。"""
        for queue in self.lane_queues.values():
            if veh_id in queue:
                queue.remove(veh_id)
                return

    def _get_congestion_point(self) -> float:
        """目標車線で連続 MIN_CONGESTED_VEHICLES 台が低速なら、その末尾位置を渋滞末尾とみなす（tail 指標用）。"""
        lane2_vehicles = get_lane_last_step_veh_ids(f"{self.env.mainlane_edge}_2")
        if len(lane2_vehicles) < MIN_CONGESTED_VEHICLES:
            return self.env.mainlane_length

        sorted_vehicles = sorted(lane2_vehicles, key=get_veh_lane_position, reverse=True)
        congested_sequence: list[str] = []
        tail_position = self.env.mainlane_length
        for vid in sorted_vehicles:
            if get_veh_speed(vid) <= CONGESTION_SPEED:
                congested_sequence.append(vid)
                if len(congested_sequence) >= MIN_CONGESTED_VEHICLES:
                    tail_position = get_veh_lane_position(congested_sequence[-1])
            else:
                congested_sequence = []
        return tail_position

    def _check_collision(self) -> None:
        """衝突を検出して記録（重複記録は抑制）。"""
        colliding_ids: list[str] = get_colliding_veh_id_list()
        if not colliding_ids:
            return
        collision_time = get_sim_time() - 0.1
        for time_val, vehicles in self.collision_history:
            if abs(time_val - collision_time) < 1.0 and set(vehicles) == set(colliding_ids):
                return
        self.collision_history.append((collision_time, colliding_ids))
        print(f"Collision detected at {collision_time:.1f} between: {', '.join(colliding_ids)}")

    def _should_continue(self) -> bool:
        sumo_time = get_sim_time()
        if sumo_time % 10 == 0:
            print("====================================================")
            print("TIME:", sumo_time, " Now:", datetime.now().time())
            print("====================================================")
        return sumo_time < self.simulation_time

    def _print_simulation_info(self, running_list: list[str]) -> None:
        print("=====================================")
        print("simulation end")
        print("total generated vehicles :", self.veh_id)
        print("running vehicles :", len(running_list))
        print("exit vehicles :", len(self.exit_vehicles))
        print("total departed vehicles :", len(self.total_departed))
        print(f"traffic volume: {len(self.total_departed) * (3600 / self.simulation_time)} pcu/h")
        print("canceled vehicles :", len(self.canceled_vehicles))
        print("=====================================")

    def _print_collision_summary(self) -> tuple[int, int]:
        total_collisions = len(self.collision_history)
        total_vehicles_involved = sum(len(vehicles) for _, vehicles in self.collision_history)
        print("\n=== Collision Summary ===")
        print(f"Total collision events: {total_collisions}")
        print(f"Total vehicles involved: {total_vehicles_involved}")
        for time_val, vehicles in self.collision_history:
            print(f"Time {time_val:.1f}: Collision between vehicles: {', '.join(vehicles)}")
        return total_collisions, total_vehicles_involved

    def _write_tail_csv(self, seed: str, tail_position_list: list[tuple[float, float]]) -> None:
        tail_csv = f"{OUTPUT_DIR}/tail_positions_{self.env.name}_through{self.inflow_through}_mlc{self.inflow_mlc}_seed{seed}.csv"
        with open(tail_csv, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["time", "tail_position"])
            writer.writerows(tail_position_list)
