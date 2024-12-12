import optparse
import os
import random
import sys
from datetime import datetime

import traci
from func.cav import CAV
from sumolib import checkBinary

simulation_time = 800.0

veh_id = 0
alpha = 0.0

departTime_r_pass = []
departTime_r_exit = []

vehicle_instance = []
total_departed_vehicle = []
exit_vehicle = []
canceled_vehicle = []

lane0_queue = []
lane1_queue = []
lane2_queue = []


def run(alpha=0.0, inflow_pass=750, inflow_exit=750):
    set_environment(inflow_pass, inflow_exit)

    while shouldContinueSimWithSimulationTime():
        traci.simulationStep()

        # このstepでシミュレーション範囲を出た車輌のリスト
        arrived_list = traci.simulation.getArrivedIDList()

        # このstepでシミュレーション範囲に入った車輌のリスト
        departed_list = traci.simulation.getDepartedIDList()

        # このstepで走行中の車輌のリスト
        running_list = traci.vehicle.getIDList()

        poplist = []

        for index, ins in enumerate(vehicle_instance):
            # シミュレーション範囲を出た車両をリスト化
            if ins.id in arrived_list:
                poplist.append(index)
                exit_vehicle.append(ins.id)
                continue
            # 混雑でまだ道路に入れていない車両はcontinue
            elif ins.id not in running_list:
                # キャンセルリストに入っていない場合は追加
                if ins.id not in canceled_vehicle:
                    canceled_vehicle.append(ins.id)
                continue

            if ins.id in departed_list:
                total_departed_vehicle.append(ins.id)
                if ins.id in canceled_vehicle:
                    canceled_vehicle.remove(ins.id)

            # 自車両の情報（位置や速度）を更新
            ins.updateStatus(running_list)

            # Laneごとのキューから車両を削除
            if ins.id in lane0_queue:
                lane0_queue.remove(ins.id)
            elif ins.id in lane1_queue:
                lane1_queue.remove(ins.id)
            elif ins.id in lane2_queue:
                lane2_queue.remove(ins.id)

            # if ins.leadPath:
            #     print(
            #         "\tvehid:",
            #         ins.id,
            #         "status",
            #         ins.status,
            #         "current speed",
            #         "{:.5g}".format(ins.speed),
            #         "route",
            #         ins.route,
            #         "road",
            #         ins.road,
            #         "leader",
            #         ins.leader,
            #         "pos x,y",
            #         "{:.5g}".format(ins.pos_x),
            #         "{:.5g}".format(ins.pos_y),
            #         "leadPath",
            #         ins.leadPath.pathID,
            #     )
            # else:
            #     print(
            #         "\tvehid:",
            #         ins.id,
            #         "status",
            #         ins.status,
            #         "current speed",
            #         "{:.5g}".format(ins.speed),
            #         "route",
            #         ins.route,
            #         "road",
            #         ins.road,
            #         "leader",
            #         ins.leader,
            #         "pos x,y",
            #         "{:.5g}".format(ins.pos_x),
            #         "{:.5g}".format(ins.pos_y),
            #     )

            # 車両の速度を更新
            ins.executionDrive()

            # 車線変更を実行
            # ins.judgeAndDoLaneChange()

        # 車両インスタンスを削除
        if poplist:
            for i in sorted(poplist, reverse=True):
                vehicle_instance.pop(i)

        # 車両の追加
        add_vehicle(alpha)

    # 生成された車輌インスタンスの数
    print("vehicle_instance Length", veh_id + 1)

    # 最後まで環境に残っている車輌の数
    print("running_list Length", len(running_list))

    # シミュレーション中に正常に終了した車両の数
    print("exit_vehicle Length", len(exit_vehicle))

    # シミュレーションに入った車輌の数
    print("total_departed_vehicle Length", len(total_departed_vehicle))

    # シミュレーション中に混雑で道路に入れなかった車両の数
    print("canceled_vehicle Length", len(canceled_vehicle))

    print("lane0_queue Length", len(lane0_queue))
    print("lane1_queue Length", len(lane1_queue))
    print("lane2_queue Length", len(lane2_queue))

    traci.close()


# シミュレーションを開始する
def startSim():
    traci.start([sumoBinary, "-c", "../config/high-way.sumocfg"])
    print("Simulation started")


# 初期設定（車両の流入時間の設定）
def set_environment(inflow_pass: int, inflow_exit: int):
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


# 車輌が侵入するレーンをランダムに決定
def getDepartLane(edge_id):
    lanes = traci.edge.getLaneNumber(edge_id)
    # 除外するレーンを指定
    exclude_lanes = []  # 路肩がないので除外なし
    available_lanes = [str(i) for i in range(lanes) if str(i) not in exclude_lanes]

    # 各レーンのキューの長さを取得
    queue_length = {"0": len(lane0_queue), "1": len(lane1_queue), "2": len(lane2_queue)}
    print("=====================================")
    print("queue_length", queue_length)
    print("=====================================")

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

    print("departLane", departLane)
    return departLane


def shouldContinueSimWithVehiclesCount():
    numVehicles = traci.simulation.getMinExpectedNumber()
    return True if numVehicles > 0 else False


def shouldContinueSimWithSimulationTime():
    sumo_time = traci.simulation.getTime()
    if sumo_time % 1 == 0:
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


def add_vehicle(alpha):
    global veh_id
    global departTime_r_pass, departTime_r_exit
    sumo_time = traci.simulation.getTime()

    alpha = float(alpha)

    # if sumo_time in departTime_r_pass:
    if sumo_time in departTime_r_pass:
        departLane = getDepartLane("MainLane1")
        traci.vehicle.add(
            vehID=str(veh_id),
            routeID="r_pass",
            typeID="CAV",
            departLane=departLane,
            departPos="base",
            departSpeed="last",
        )
        instance = CAV(veh_id, alpha, withAgree=True)
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
        departLane = getDepartLane("MainLane1")
        traci.vehicle.add(
            vehID=str(veh_id),
            routeID="r_exit",
            typeID="CAV",
            departLane=departLane,
            departPos="base",
            departSpeed="last",
        )
        instance = CAV(veh_id, alpha, withAgree=True)
        vehicle_instance.append(instance)

        if departLane == "0":
            lane0_queue.append(str(veh_id))
        elif departLane == "1":
            lane1_queue.append(str(veh_id))
        elif departLane == "2":
            lane2_queue.append(str(veh_id))

        veh_id += 1


def get_options():
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


if __name__ == "__main__":
    # コマンドライン引数を取得
    options = get_options()
    args = sys.argv
    alpha = str(args[1])  #
    seed = args[2]  # 乱数のシード(等しいseedで実行すると同じ結果が得られる)
    random.seed(seed)
    inflow_pass = int(args[3])  # 車両の流入数 pass
    inflow_exit = int(args[4])  # 車両の流入数 exit

    # this script has been called from the command line. It will start sumo as a server, then connect and run
    if options.nogui:
        sumoBinary = checkBinary("sumo")
    else:
        sumoBinary = checkBinary("sumo-gui")

    startSim()
    run(alpha, inflow_pass, inflow_exit)
