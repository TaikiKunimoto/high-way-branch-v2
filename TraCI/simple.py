"""
提案の相手となるモデル, 車線変更開始位置は固定 & 車線変更に優先度はない
"""

import csv
import optparse
import os
import random
import sys
from datetime import datetime

import matplotlib.pyplot as plt
import numpy as np
import traci
from func.simple_cav import SimpleCAV
from simulationStatistics.simulation_statistics import SimulationStatistics
from sumolib import checkBinary

simulation_time = 600.0

veh_id = 0

departTime_r_pass = []
departTime_r_exit = []

vehicle_instance = []
total_departed_vehicle = []
exit_vehicle = []
r_pass_departed_vehicle = []
r_exit_departed_vehicle = []
r_pass_exit_vehicle = []
r_exit_exit_vehicle = []
canceled_vehicle = []
canceled_veh_without_collied_veh = []
collision_history = []  # 各要素は (time, vehicle_id1, vehicle_id2) のタプル
total_collisions = 0
total_vehicles_involved = 0

# タイムスペース図記録用リスト
lane_data = {"lane0": [], "lane1": [], "lane2": []}  # 各車線ごとのデータを保持

lane0_queue = []
lane1_queue = []
lane2_queue = []

CONGESTION_SPEED = 11.1  # m/s (40 km/h) 渋滞判定の速度
LANE2_MIN_CONGESTED_VEHICLES = 5  # Lane2での渋滞判定の最低車両数
LANE1_MIN_CONGESTED_VEHICLES = 5  # Lane1での渋滞判定の最低車両数
MAINLANE_LENGTH = 2500  # m


def run(inflow_pass, inflow_exit):
    _set_environment(inflow_pass, inflow_exit)

    last_recorded_sec = 0
    tail_position_list = []
    max_tail_position = 0

    while _shouldContinueSimWithSimulationTime():
        traci.simulationStep()

        _check_collision()

        # step毎の車線変更履歴を初期化
        lane_change_history = {}

        # このstepでシミュレーション範囲を出た車輌のリスト
        arrived_list = traci.simulation.getArrivedIDList()
        # このstepでシミュレーション範囲に入った車輌のリスト
        departed_list = traci.simulation.getDepartedIDList()
        # このstepで走行中の車輌のリスト
        running_list = traci.vehicle.getIDList()

        poplist = []

        lane_2_congestion_tail_point = _getLane2CongestionPoint()
        lane_1_congestion_head_point = _getLane1CongestionPoint()

        # tail position を記録
        current_time = traci.simulation.getTime()
        current_sec = int(current_time)
        if current_sec != last_recorded_sec:
            tail_pos = MAINLANE_LENGTH - lane_2_congestion_tail_point
            tail_position_list.append((current_time, tail_pos))
            if tail_pos > max_tail_position:
                max_tail_position = tail_pos
            last_recorded_sec = current_sec

        for index, ins in enumerate(vehicle_instance):
            # シミュレーション範囲を出た車両をリスト化
            if ins.id in arrived_list:
                poplist.append(index)
                exit_vehicle.append(ins.id)

                ins.get_arrival_time()
                # 車輌の travel time と average speed を計算
                if ins.route == "r_pass":
                    stats.calculate_travel_time(
                        "r_pass", ins.departure_time, ins.arrival_time
                    )
                    stats.calculate_vehicle_average_spped("r_pass", ins.speed_history)
                    r_pass_exit_vehicle.append(ins.id)
                elif ins.route == "r_exit":
                    stats.calculate_travel_time(
                        "r_exit", ins.departure_time, ins.arrival_time
                    )
                    stats.calculate_vehicle_average_spped("r_exit", ins.speed_history)
                    r_exit_exit_vehicle.append(ins.id)
                continue

            # 混雑でまだ道路に入れていない車両はcontinue
            elif ins.id not in running_list:
                # キャンセルリストに入っていない場合は追加
                if ins.id not in canceled_vehicle:
                    canceled_vehicle.append(ins.id)
                continue

            # シミュレーション範囲に入った車両をリスト化し、キャンセルリストから削除
            if ins.id in departed_list:
                ins.get_departure_time()
                total_departed_vehicle.append(ins.id)

                if ins.route == "r_pass":
                    r_pass_departed_vehicle.append(ins.id)
                elif ins.route == "r_exit":
                    r_exit_departed_vehicle.append(ins.id)

                if ins.id in canceled_vehicle:
                    canceled_vehicle.remove(ins.id)

            # 自車両の情報（位置や速度）を更新
            ins.updateStatus()
            # 自身の行動(Priority)を更新
            ins.decideNextActionAndPriority()
            # 車線変更を実行
            ins.executeLaneChange(
                lane_change_history,
                lane_2_congestion_tail_point,
                lane_1_congestion_head_point,
            )
            # 車両の速度を更新
            ins.controlSpeed()

            # use in debug
            # if ins.id == "0" or ins.id == "1" or ins.id == "2":
            #     print(
            #         f"Vehicle ID: {ins.id}, Route: {ins.route}, lane : {ins.laneID}, pos: {ins.pos_x} action: {ins.action}, priority: {ins.priority}, status: {ins.status}"
            #     )

            # Laneごとのキューから車両を削除
            _updateLaneQueue(ins.id)

            # TTCを計算
            if ins.leader_distance is not None:
                stats.calculate_TTC(ins.leader_distance, ins.leader_speed, ins.speed)

            _record_lane_data(ins.laneID, ins.lane_pos, ins.speed)

        # 車両インスタンスを削除
        if poplist:
            for i in sorted(poplist, reverse=True):
                vehicle_instance.pop(i)

        # 車両の追加
        _add_vehicle()

    collided_vehicles = set()
    for _, vehicles in collision_history:
        collided_vehicles.update(vehicles)

    canceled_veh_without_collied_veh = [
        veh_id for veh_id in canceled_vehicle if veh_id not in collided_vehicles
    ]

    _printSImulationInfoAtEnd(running_list)
    total_collisions, total_vehicles_involved = _print_collision_summary()

    # tail_position_list を CSV に保存
    tail_position_csv = f"simulationStatistics/statistics/simple/tail_positions_pass{inflow_pass}_exit{inflow_exit}_seed{seed}.csv"
    with open(tail_position_csv, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["time", "tail_position"])
        for time_step, tail_pos in tail_position_list:
            writer.writerow([time_step, tail_pos])

    # タイムスペース図をプロット
    _plot_time_space_diagram()

    # シミュレーション結果をcsvファイルに保存
    results = {
        "total_generated_vehicle": veh_id,
        "total_departed_vehicle": total_departed_vehicle,
        "running_vehicle": running_list,
        "exit_vehicle": exit_vehicle,
        "r_pass_departed_vehicle": r_pass_departed_vehicle,
        "r_exit_departed_vehicle": r_exit_departed_vehicle,
        "r_pass_exit_vehicle": r_pass_exit_vehicle,
        "r_exit_exit_vehicle": r_exit_exit_vehicle,
        "canceled_vehicle": canceled_veh_without_collied_veh,
        # "lane0_queue": lane0_queue,
        # "lane1_queue": lane1_queue,
        # "lane2_queue": lane2_queue,
        "traffic_volume": len(total_departed_vehicle) * (3600 / simulation_time),
        "total_collisions": total_collisions,
        "total_vehicles_involved": total_vehicles_involved,
        "max_tail_position": max_tail_position,
    }
    stats.add_result(simulation_time, seed, inflow_pass, inflow_exit, results)

    traci.close()


# シミュレーションを開始する
def _startSim():
    traci.start([sumoBinary, "-c", "../config/high-way.sumocfg"])
    print("Simulation started")


# 初期設定（車両の流入時間の設定）
def _set_environment(inflow_pass: int, inflow_exit: int):
    global vehicle_instance
    global veh_id
    global departTime_r_pass, departTime_r_exit

    k_0 = int((simulation_time / 3600) * inflow_pass)
    k_1 = int((simulation_time / 3600) * inflow_exit)

    # 車両の流入時刻を決定
    departTime_r_pass = sorted(random.sample(range(int(simulation_time)), k_0))
    departTime_r_exit = sorted(random.sample(range(int(simulation_time)), k_1))

    # 1秒以上開けて流入
    departTime_r_pass = [round(n, 1) + 0.1 for n in departTime_r_pass]
    departTime_r_exit = [round(n, 1) + 0.1 for n in departTime_r_exit]

    print("deparTime_r_pass", departTime_r_pass)
    print("departTime_r_exit", departTime_r_exit)


def _printSImulationInfoAtEnd(running_list):
    print("=====================================")
    print("simulation end")
    # 生成された車輌インスタンスの数
    print("vehicle_instance Length :", veh_id)
    # 最後までシミュレーション内部に残っている車輌の数
    print("running_list Length :", len(running_list))
    # シミュレーション中に正常に終了した車両の数
    print("exit_vehicle Length :", len(exit_vehicle))
    # シミュレーションに入った車輌の数
    print("total_departed_vehicle Length :", len(total_departed_vehicle))
    # １時間あたりの交通量
    print(
        f"traffic volume: {len(total_departed_vehicle) * (3600 / simulation_time)} pcu/h"
    )
    # シミュレーション中に混雑で道路に入れなかった車両の数
    print("canceled_vehicle Length :", len(canceled_veh_without_collied_veh))
    # シミュレーション終了時の各レーンのキューの長さ
    print("lane0_queue Length :", len(lane0_queue))
    print("lane1_queue Length :", len(lane1_queue))
    print("lane2_queue Length :", len(lane2_queue))
    print("=====================================")


def _print_collision_summary():
    total_collisions = len(collision_history)
    total_vehicles_involved = sum(len(vehicles) for _, vehicles in collision_history)

    print("\n=== Collision Summary ===")
    print(f"Total number of collision events: {total_collisions}")
    print(f"Total number of vehicles involved in collisions: {total_vehicles_involved}")
    print("\nCollision details:")
    for time, vehicles in collision_history:
        print(f"Time {time:.1f}: Collision between vehicles: {', '.join(vehicles)}")

    return total_collisions, total_vehicles_involved


def _check_collision():
    colliding_ids = traci.simulation.getCollidingVehiclesIDList()
    if len(colliding_ids) > 0:
        collision_time = traci.simulation.getTime()

        # 同じ衝突が重複して記録されないようにチェック
        for time, vehicles in collision_history:
            if abs(time - collision_time) < 1.0 and set(vehicles) == set(colliding_ids):
                return

        collision_history.append((collision_time, colliding_ids))
        print(
            f"Collision detected at time {collision_time:.1f} between vehicles: {', '.join(colliding_ids)}"
        )


def _updateLaneQueue(id: str):
    if id in lane0_queue:
        lane0_queue.remove(id)
    elif id in lane1_queue:
        lane1_queue.remove(id)
    elif id in lane2_queue:
        lane2_queue.remove(id)


# 車輌が侵入するレーンをランダムに決定
def _getDepartLane(edge_id):
    lanes = traci.edge.getLaneNumber(edge_id)
    # 除外するレーンを指定
    exclude_lanes = []  # 路肩がないので除外なし
    available_lanes = [str(i) for i in range(lanes) if str(i) not in exclude_lanes]

    # 各レーンのキューの長さを取得
    queue_length = {"0": len(lane0_queue), "1": len(lane1_queue), "2": len(lane2_queue)}

    # キャンセルキューがないレーンを取得
    lanes_without_queue = [lane for lane in available_lanes if queue_length[lane] == 0]

    if lanes_without_queue:
        # キャンセルキューがないレーンがあればその中からランダムに選択
        departLane = random.choice(lanes_without_queue)
    else:
        # 最小キューのレーンを取得し、その中からランダムに選択
        min_queue_length = min(queue_length.values())
        min_queue_lanes = [
            lane for lane, length in queue_length.items() if length == min_queue_length
        ]
        departLane = random.choice(min_queue_lanes)

    return departLane


# Lane2での渋滞が発生しているポイントを調査
def _getLane2CongestionPoint():
    lane2_vehicles = traci.lane.getLastStepVehicleIDs("MainLane1_2")
    if len(lane2_vehicles) < LANE2_MIN_CONGESTED_VEHICLES:
        return MAINLANE_LENGTH

    sorted_vehicles = sorted(
        lane2_vehicles,
        key=lambda x: traci.vehicle.getLanePosition(x),
        reverse=True,
    )

    congested_sequence = []
    tail_position = MAINLANE_LENGTH
    for veh_id in sorted_vehicles:
        speed = traci.vehicle.getSpeed(veh_id)

        if speed <= CONGESTION_SPEED:
            congested_sequence.append(veh_id)
            if len(congested_sequence) >= LANE2_MIN_CONGESTED_VEHICLES:
                tail_position = traci.vehicle.getLanePosition(congested_sequence[-1])
                continue
        else:
            congested_sequence = []
            continue

    return tail_position


# Lane1での渋滞が発生しているポイントを調査
def _getLane1CongestionPoint():
    lane1_vehicles = traci.lane.getLastStepVehicleIDs("MainLane1_1")
    if len(lane1_vehicles) < LANE1_MIN_CONGESTED_VEHICLES:
        return None

    sorted_vehicles = sorted(
        lane1_vehicles,
        key=lambda x: traci.vehicle.getLanePosition(x),
        reverse=True,
    )

    congested_sequence = []
    head_position = None
    for veh_id in sorted_vehicles:
        speed = traci.vehicle.getSpeed(veh_id)

        if speed <= CONGESTION_SPEED:
            congested_sequence.append(veh_id)
            if len(congested_sequence) >= LANE1_MIN_CONGESTED_VEHICLES:
                head_position = traci.vehicle.getLanePosition(congested_sequence[0])
                continue
        else:
            congested_sequence = []
            continue

    return head_position


def _shouldContinueSimWithVehiclesCount():
    numVehicles = traci.simulation.getMinExpectedNumber()
    return True if numVehicles > 0 else False


def _shouldContinueSimWithSimulationTime():
    sumo_time = traci.simulation.getTime()
    if sumo_time % 10 == 0:
        now = datetime.now().time()
        print(
            "====================================================",
            "\nTIME:",
            sumo_time,
            "\nNow:",
            now,
            "\n====================================================",
        )
    return True if sumo_time < simulation_time else False


def _add_vehicle():
    global veh_id
    global departTime_r_pass, departTime_r_exit
    sumo_time = traci.simulation.getTime()

    # if sumo_time in departTime_r_pass:
    if sumo_time in departTime_r_pass:
        departLane = _getDepartLane("MainLane1")
        traci.vehicle.add(
            vehID=str(veh_id),
            routeID="r_pass",
            typeID="CAV",
            departLane=departLane,
            departPos="base",
            departSpeed="last",
        )
        instance = SimpleCAV(veh_id)
        vehicle_instance.append(instance)

        if departLane == "0":
            lane0_queue.append(str(veh_id))
        elif departLane == "1":
            lane1_queue.append(str(veh_id))
        elif departLane == "2":
            lane2_queue.append(str(veh_id))

        veh_id += 1

    # if sumo_time in departTime_r_exit:
    if sumo_time in departTime_r_exit:
        departLane = _getDepartLane("MainLane1")
        traci.vehicle.add(
            vehID=str(veh_id),
            routeID="r_exit",
            typeID="CAV",
            departLane=departLane,
            departPos="base",
            departSpeed="last",
        )
        instance = SimpleCAV(veh_id)
        vehicle_instance.append(instance)

        if departLane == "0":
            lane0_queue.append(str(veh_id))
        elif departLane == "1":
            lane1_queue.append(str(veh_id))
        elif departLane == "2":
            lane2_queue.append(str(veh_id))

        veh_id += 1


def _record_lane_data(lane_id, pos, speed):
    if seed != "1":
        return

    current_time = traci.simulation.getTime()

    if "MainLane1_0" in lane_id:
        lane_data["lane0"].append((current_time, pos, speed))
    elif "MainLane1_1" in lane_id:
        lane_data["lane1"].append((current_time, pos, speed))
    elif "MainLane1_2" in lane_id:
        lane_data["lane2"].append((current_time, pos, speed))


# タイムスペース図のプロット
def _plot_time_space_diagram(output_dir="simulationStatistics/statistics/simple"):
    if seed != "1":
        return
    # 保存先ディレクトリが存在しない場合は作成
    os.makedirs(output_dir, exist_ok=True)

    for lane, data in lane_data.items():
        if not data:
            continue
        times, positions, speeds = zip(*data)
        plt.figure(figsize=(12, 6))
        sc = plt.scatter(times, positions, c=speeds, cmap="jet_r", s=1)
        plt.colorbar(sc, label="Speed (m/s)")
        plt.xlabel("Time (s)")
        plt.ylabel("Position (m)")
        plt.title(f"Time-Space Diagram for {lane}")

        # ファイル保存
        output_path = os.path.join(
            output_dir,
            f"simple_{inflow_pass}_{inflow_exit}_{lane}_time_space_diagram.png",
        )
        plt.savefig(output_path)
        plt.close()


def _get_options():
    # define options for this script and interpret the command line
    optParser = optparse.OptionParser()
    optParser.add_option(
        "--nogui",
        action="store_true",
        default=False,
        help="run the commandline version of sumo",
    )
    options, args = optParser.parse_args()
    return options


def _create_file_name():
    return f"simple_pass{inflow_pass}_exit{inflow_exit}_seed{seed}"


if __name__ == "__main__":
    # コマンドライン引数を取得
    options = _get_options()
    args = sys.argv
    seed = args[1]  # 乱数のシード(等しいseedで実行すると同じ結果が得られる)
    random.seed(seed)
    inflow_pass = int(args[2])  # 車両の流入数 pass
    inflow_exit = int(args[3])  # 車両の流入数 exit

    filename = _create_file_name()
    stats = SimulationStatistics(
        filename=filename, output_dir="simulationStatistics/statistics/simple"
    )

    # this script has been called from the command line. It will start sumo as a server, then connect and run
    if options.nogui:
        sumoBinary = checkBinary("sumo")
    else:
        sumoBinary = checkBinary("sumo-gui")

    _startSim()
    run(inflow_pass, inflow_exit)
