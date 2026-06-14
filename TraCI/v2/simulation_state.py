"""v2 のシミュレーション状態と毎step メインループ（2フェーズの骨格）。

B1 では「流入・車両追加削除・到着処理・統計配線・traci 起動」のパイプラインを通すところまで。
車線変更（Layer2）・調停（Layer1）は ``# B2+`` の位置に後続ブランチで差し込む。
``custom.py`` の run() を踏襲しつつ V2CAV を用い、タイムスペース図(matplotlib)出力は省略する。
"""

import csv
from datetime import datetime
import os
import random
import sys

from simulationStatistics.simulation_statistics import SimulationStatistics
from status.status import CarStatus
from utils.traci_wrapper import (
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
from v2.lc_request import LCRequest, build_requests, in_activation_window
from v2.pair_executor import execute_pairs
from v2.priority import Key, order_requests
from v2.rsu import Assignment, arbitrate
from v2.snapshot import Snapshot, capture
from v2.v2_cav import V2CAV

if "SUMO_HOME" in os.environ:
    tools = os.path.join(os.environ["SUMO_HOME"], "tools")
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")
import traci

OUTPUT_DIR = "simulationStatistics/statistics/v2"


class V2SimulationState:
    """1回のシミュレーションを通じて保持する状態。"""

    def __init__(self, simulation_time: float, env: Environment) -> None:
        self.simulation_time: float = simulation_time
        self.env: Environment = env
        self.veh_id: int = 0
        # グループ別の流入時刻（環境のグループ定義順を保持）
        self.group_depart_times: list[tuple[Group, list[float]]] = []
        self.inflow_through: int = 0  # CSV 用: 必須LCなし車の流入量
        self.inflow_mlc: int = 0  # CSV 用: 必須LC車の流入量
        self.vehicles: list[V2CAV] = []
        self.total_departed: list[str] = []
        self.exit_vehicles: list[str] = []
        self.canceled_vehicles: list[str] = []
        self.collision_history: list[tuple[float, list[str]]] = []
        # 各車線の待ち行列（流入レーン選択の負荷分散に使う）。レーンは環境により可変なので動的に作る
        self.lane_queues: dict[str, list[str]] = {}


def run(
    state: V2SimulationState, total_inflow: float, mlc_ratio: float, stats: SimulationStatistics, seed: str
) -> None:
    _set_environment(state, total_inflow, mlc_ratio)

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

    while _should_continue(state):
        traci.simulationStep()
        _check_collision(state)

        arrived_list = get_sim_arrived_veh_id_list()
        departed_list = get_sim_departed_veh_id_list()
        running_list = get_veh_id_list()

        # tail position の記録（タイムスペース指標用）
        current_time = get_sim_time()
        current_sec = int(current_time)
        if current_sec != last_recorded_second:
            tail_pos = state.env.mainlane_length - _get_congestion_point(state.env)
            tail_position_list.append((current_time, tail_pos))
            max_tail_position = max(max_tail_position, tail_pos)
            last_recorded_second = current_sec

        # --- 到着/未発進/出発処理 と 観測。全車を先に観測し、スナップショット S_t の一貫性を保つ ---
        poplist: list[int] = []
        active: list[V2CAV] = []
        for index, veh in enumerate(state.vehicles):
            vid = veh.params.id

            # シミュレーション範囲を出た車両
            if vid in arrived_list:
                poplist.append(index)
                state.exit_vehicles.append(vid)
                veh.record_arrival_time()
                _accumulate_exit_stats(veh, stats, r_pass_exited, r_exit_exited)
                continue

            # 混雑で未発進の車両
            if vid not in running_list:
                if vid not in state.canceled_vehicles:
                    state.canceled_vehicles.append(vid)
                continue

            # 発進した車両の出発時刻記録
            if vid in departed_list:
                veh.record_departure_time()
                state.total_departed.append(vid)
                if veh.params.target_lane is None:  # 必須LCなし（through）
                    r_pass_departed.append(vid)
                else:  # 必須LC車
                    r_exit_departed.append(vid)
                if vid in state.canceled_vehicles:
                    state.canceled_vehicles.remove(vid)

            veh.update_observation()
            _update_activation(veh, state.env.mainlane_edge)
            active.append(veh)
            _update_lane_queue(state, vid)

            if veh.params.leader_distance is not None and veh.params.leader_speed is not None:
                stats.calculate_TTC(veh.params.leader_distance, veh.params.leader_speed, veh.params.speed)

        # --- 毎Tc 2フェーズ調停。Phase A（鍵計算）→ Phase B（割当＋役割付与）。Layer2 実行は制御後に行う ---
        tc_accumulator += TIME_STEP
        if tc_accumulator + 1e-9 >= TC:
            tc_accumulator = 0.0
            snap = capture(active, current_time, state.env.mainlane_edge)
            requests = build_requests(snap)
            keyed = order_requests(requests)  # Phase A: 全要求車の鍵を計算し EDF（dist昇順）にソート
            assignments = arbitrate(keyed, snap)  # Phase B: 鍵順に提供車を占有印つきで確保
            req_by_id = {r.veh_id: r for _, r in keyed}
            _apply_roles(active, assignments)  # 毎Tc フル再構築（提供車=YIELDING / 要求車=LANE_CHANGING）
            if not _keys_unique(keyed):
                tie_events += 1
            if not _providers_unique(assignments):
                double_assign_events += 1
            if current_sec % 50 == 0 and current_sec != last_request_log_sec:
                _log_assignments(current_time, keyed, assignments)
                last_request_log_sec = current_sec

        # --- 制御（速度）。traci の速度指令は次 step に反映されるため観測順と独立 ---
        for veh in active:
            veh.control_speed()

        # --- Layer2 実行。制御後に呼び、協調減速の slowDown と changeLane が最後の指令になるようにする ---
        if snap is not None:
            total_lc += execute_pairs(assignments, req_by_id, snap, {veh.params.id: veh for veh in active})

        for i in sorted(poplist, reverse=True):
            state.vehicles.pop(i)

        _add_vehicle(state)

    # 終了時、残車両の統計を更新
    for veh in state.vehicles:
        if veh.params.id not in running_list:
            continue
        if veh.params.target_lane is None:  # 必須LCなし（through）
            stats.calculate_vehicle_average_speed("r_pass", veh.params.speed_history)
        else:  # 必須LC車
            stats.calculate_vehicle_average_speed("r_exit", veh.params.speed_history)
            if veh.params.pos_x is not None:
                r_exit_running[veh.params.id] = veh.params.pos_x

    r_exit_running_list = sorted(r_exit_running, key=lambda v: r_exit_running[v], reverse=True)

    collided: set[str] = set()
    for _, vehicles in state.collision_history:
        collided.update(vehicles)
    canceled_without_collision = [v for v in state.canceled_vehicles if v not in collided]

    _print_simulation_info(state, running_list)
    print(f"Phase A: Tc rounds with key ties (should be 0): {tie_events}")
    print(f"Phase B: Tc rounds with double-assigned providers (should be 0): {double_assign_events}")
    print(f"Layer2: total instant lane changes executed: {total_lc}")
    total_collisions, total_involved = _print_collision_summary(state)
    _write_tail_csv(state.env.name, state.inflow_through, state.inflow_mlc, seed, tail_position_list)

    results = {
        "total_generated_vehicle": state.veh_id,
        "total_departed_vehicle": state.total_departed,
        "running_vehicle": running_list,
        "exit_vehicle": state.exit_vehicles,
        "r_pass_departed_vehicle": r_pass_departed,
        "r_exit_departed_vehicle": r_exit_departed,
        "r_pass_exit_vehicle": r_pass_exited,
        "r_exit_exit_vehicle": r_exit_exited,
        "r_exit_running_vehicle": r_exit_running_list,
        "canceled_vehicle": canceled_without_collision,
        "traffic_volume": len(state.total_departed) * (3600 / state.simulation_time),
        "total_collisions": total_collisions,
        "total_vehicles_involved": total_involved,
        "max_tail_position": max_tail_position,
    }
    stats.add_result(state.simulation_time, seed, state.inflow_through, state.inflow_mlc, results)
    traci.close()


def _accumulate_exit_stats(
    veh: V2CAV, stats: SimulationStatistics, r_pass_exited: list[str], r_exit_exited: list[str]
) -> None:
    """範囲外に出た車両の走行時間・平均速度を統計に加算する（through=r_pass バケツ／必須LC車=r_exit バケツ）。"""
    p = veh.params
    if p.target_lane is None:  # 必須LCなし（through）
        if p.departure_time is not None and p.arrival_time is not None:
            stats.calculate_travel_time("r_pass", p.departure_time, p.arrival_time)
        stats.calculate_vehicle_average_speed("r_pass", p.speed_history)
        r_pass_exited.append(p.id)
    else:  # 必須LC車
        if p.departure_time is not None and p.arrival_time is not None:
            stats.calculate_travel_time("r_exit", p.departure_time, p.arrival_time)
        stats.calculate_vehicle_average_speed("r_exit", p.speed_history)
        r_exit_exited.append(p.id)


def _update_activation(veh: V2CAV, mainlane_edge: str) -> None:
    """必須LC要求の活性化窓に初めて入った時刻を記録する（早め固定活性化、一度だけ）。"""
    p = veh.params
    if p.activated:
        return
    if in_activation_window(mainlane_edge, p.road, p.target_lane, p.deadline_pos, p.lane, p.lane_pos):
        p.activated = True
        p.activation_time = p.sim_time


def _apply_roles(active: list[V2CAV], assignments: list[Assignment]) -> None:
    """毎Tc フル再構築: 全車の役割を NORMAL にリセットしてから割当結果を反映する。"""
    by_id = {veh.params.id: veh for veh in active}
    for veh in active:
        veh.params.status = CarStatus.NORMAL
        veh.params.providing_to_id = None
        veh.params.receiving_from_id = None
    for a in assignments:
        requester = by_id.get(a.requester_id)
        provider = by_id.get(a.provider_id)
        if requester is None or provider is None:
            continue
        requester.params.status = CarStatus.LANE_CHANGING
        requester.params.receiving_from_id = a.provider_id
        provider.params.status = CarStatus.YIELDING
        provider.params.providing_to_id = a.requester_id


def _keys_unique(keyed: list[tuple[Key, LCRequest]]) -> bool:
    """鍵がすべて相異なるか（=同点なし）。ID が一意なので常に True のはず（デッドロックフリー）。"""
    keys = [k for k, _ in keyed]
    return len(set(keys)) == len(keys)


def _providers_unique(assignments: list[Assignment]) -> bool:
    """同一提供車が複数の要求車に割り当たっていないか（横取り禁止なら常に True）。"""
    providers = [a.provider_id for a in assignments]
    return len(set(providers)) == len(providers)


def _log_assignments(sim_time: float, keyed: list[tuple[Key, LCRequest]], assignments: list[Assignment]) -> None:
    """Phase A/B の結果（EDF順とどの要求車が提供車を得たか）をログ出力する（B4 の検証用）。"""
    amap = {a.requester_id: a.provider_id for a in assignments}
    print(f"[Tc t={sim_time:.1f}] requests={len(keyed)} assigned={len(assignments)}")
    for key, r in keyed:
        print(f"    veh={r.veh_id} dist={key[0]:.1f} k={r.remaining_k} <- provider={amap.get(r.veh_id, '-')}")


def _set_environment(state: V2SimulationState, total_inflow: float, mlc_ratio: float) -> None:
    """環境のグループ別流入量（総流入 Q × 必須LC比率 f から展開）に従い、流入時刻を乱数で決定（seed で決定的）。

    グループ定義順に random.sample を呼ぶことで決定性を保つ（分流D では through→exiting＝旧 pass→exit と一致）。
    """
    for group, rate in state.env.group_rates(total_inflow, mlc_ratio):
        k = int((state.simulation_time / 3600) * rate)
        seconds = sorted(random.sample(range(int(state.simulation_time)), k))
        # 1秒以上間隔を確保（同一stepへの偏りを避ける）
        times: list[float] = [round(n, 1) + 0.1 for n in seconds]
        state.group_depart_times.append((group, times))
        if group.target_lane is not None:
            state.inflow_mlc += int(rate)
        else:
            state.inflow_through += int(rate)
        print(f"depart_times[{group.name}]:", times)


def _get_depart_lane(state: V2SimulationState, edge_id: str, allowed_lanes: tuple[int, ...] | None) -> str:
    """グループの投入レーン候補（None=全レーン）の中で待ち行列が最短のレーンを選ぶ（負荷分散）。"""
    lanes_total: int = traci.edge.getLaneNumber(edge_id)
    candidates = [str(i) for i in (allowed_lanes if allowed_lanes is not None else range(lanes_total))]
    queue_length = {lane: len(state.lane_queues.get(lane, [])) for lane in candidates}
    lanes_without_queue = [lane for lane in candidates if queue_length[lane] == 0]
    if lanes_without_queue:
        return random.choice(lanes_without_queue)
    min_length = min(queue_length.values())
    min_lanes = [lane for lane, length in queue_length.items() if length == min_length]
    return random.choice(min_lanes)


def _add_vehicle(state: V2SimulationState) -> None:
    """流入時刻に到達した車両を SUMO に追加し V2CAV を生成する。各車は環境のグループから必須LC仕様を受け取る。"""
    sumo_time = get_sim_time()
    for group, depart_times in state.group_depart_times:
        if sumo_time not in depart_times:
            continue
        depart_edge = group.depart_edge if group.depart_edge is not None else state.env.mainlane_edge
        depart_lane = _get_depart_lane(state, depart_edge, group.depart_lanes)
        traci.vehicle.add(
            vehID=str(state.veh_id),
            routeID=group.route,
            typeID="CAV",
            departLane=depart_lane,
            departPos="base",
            departSpeed="last",
        )
        state.vehicles.append(V2CAV(state.veh_id, target_lane=group.target_lane, deadline_pos=group.deadline_pos))
        state.lane_queues.setdefault(depart_lane, []).append(str(state.veh_id))
        state.veh_id += 1


def _update_lane_queue(state: V2SimulationState, veh_id: str) -> None:
    """走行を開始した車両を待ち行列から外す。"""
    for queue in state.lane_queues.values():
        if veh_id in queue:
            queue.remove(veh_id)
            return


def _get_congestion_point(env: Environment) -> float:
    """目標車線で連続 MIN_CONGESTED_VEHICLES 台が低速なら、その末尾位置を渋滞末尾とみなす（tail 指標用）。"""
    lane2_vehicles = get_lane_last_step_veh_ids(f"{env.mainlane_edge}_2")
    if len(lane2_vehicles) < MIN_CONGESTED_VEHICLES:
        return env.mainlane_length

    sorted_vehicles = sorted(lane2_vehicles, key=get_veh_lane_position, reverse=True)
    congested_sequence: list[str] = []
    tail_position = env.mainlane_length
    for vid in sorted_vehicles:
        if get_veh_speed(vid) <= CONGESTION_SPEED:
            congested_sequence.append(vid)
            if len(congested_sequence) >= MIN_CONGESTED_VEHICLES:
                tail_position = get_veh_lane_position(congested_sequence[-1])
        else:
            congested_sequence = []
    return tail_position


def _check_collision(state: V2SimulationState) -> None:
    """衝突を検出して記録（重複記録は抑制）。"""
    colliding_ids: list[str] = list(traci.simulation.getCollidingVehiclesIDList())
    if not colliding_ids:
        return
    collision_time = get_sim_time() - 0.1
    for time_val, vehicles in state.collision_history:
        if abs(time_val - collision_time) < 1.0 and set(vehicles) == set(colliding_ids):
            return
    state.collision_history.append((collision_time, colliding_ids))
    print(f"Collision detected at {collision_time:.1f} between: {', '.join(colliding_ids)}")


def _should_continue(state: V2SimulationState) -> bool:
    sumo_time = get_sim_time()
    if sumo_time % 10 == 0:
        print("====================================================")
        print("TIME:", sumo_time, " Now:", datetime.now().time())
        print("====================================================")
    return sumo_time < state.simulation_time


def _print_simulation_info(state: V2SimulationState, running_list: list[str]) -> None:
    print("=====================================")
    print("simulation end")
    print("total generated vehicles :", state.veh_id)
    print("running vehicles :", len(running_list))
    print("exit vehicles :", len(state.exit_vehicles))
    print("total departed vehicles :", len(state.total_departed))
    print(f"traffic volume: {len(state.total_departed) * (3600 / state.simulation_time)} pcu/h")
    print("canceled vehicles :", len(state.canceled_vehicles))
    print("=====================================")


def _print_collision_summary(state: V2SimulationState) -> tuple[int, int]:
    total_collisions = len(state.collision_history)
    total_vehicles_involved = sum(len(vehicles) for _, vehicles in state.collision_history)
    print("\n=== Collision Summary ===")
    print(f"Total collision events: {total_collisions}")
    print(f"Total vehicles involved: {total_vehicles_involved}")
    for time_val, vehicles in state.collision_history:
        print(f"Time {time_val:.1f}: Collision between vehicles: {', '.join(vehicles)}")
    return total_collisions, total_vehicles_involved


def _write_tail_csv(
    env_name: str, inflow_through: int, inflow_mlc: int, seed: str, tail_position_list: list[tuple[float, float]]
) -> None:
    tail_csv = f"{OUTPUT_DIR}/tail_positions_{env_name}_through{inflow_through}_mlc{inflow_mlc}_seed{seed}.csv"
    with open(tail_csv, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["time", "tail_position"])
        writer.writerows(tail_position_list)
