import optparse
import os
import random
import sys
from datetime import datetime

import traci
from func.cav import CAV
from sumolib import checkBinary

simulation_time = 300.0

vehicle_instance = []
veh_id = 0
alpha = 0.0
departTime_lane0 = []
departTime_lane1 = []


def run(alpha=0.0, inflow_0=750, inflow_1=750):
    set_environment(inflow_0, inflow_1)

    while shouldContinueSimWithSimulationTime():
        traci.simulationStep()

        arrived_list = traci.simulation.getArrivedIDList()
        running_list = traci.vehicle.getIDList()
        poplist = []

        for index, ins in enumerate(vehicle_instance):
            # シミュレーション範囲を出た車両をリスト化
            if ins.id in arrived_list:
                poplist.append(index)
                continue
            # 混雑でまだ道路に入れていない車両はパス
            elif ins.id not in running_list:
                continue
        
        # 自車両の情報（位置や速度）を更新
            ins.updateStatus(running_list)
            if ins.leadPath:
                print(
                    "\tvehid:",
                    ins.id,
                    "status",
                    ins.status,
                    "current speed",
                    "{:.5g}".format(ins.speed),
                    "route",
                    ins.route,
                    "road",
                    ins.road,
                    "leader",
                    ins.leader,
                    "pos x,y",
                    "{:.5g}".format(ins.pos_x),
                    "{:.5g}".format(ins.pos_y),
                    "leadPath",
                    ins.leadPath.pathID,
                )
            else:
                print(
                    "\tvehid:",
                    ins.id,
                    "status",
                    ins.status,
                    "current speed",
                    "{:.5g}".format(ins.speed),
                    "route",
                    ins.route,
                    "road",
                    ins.road,
                    "leader",
                    ins.leader,
                    "pos x,y",
                    "{:.5g}".format(ins.pos_x),
                    "{:.5g}".format(ins.pos_y),
                )

            # 車両の速度を更新
            ins.executionDrive()

        # 車両インスタンスを削除
        if poplist:
            for i in sorted(poplist, reverse=True):
                vehicle_instance.pop(i)

        # 車両の追加
        add_vehicle(alpha)

    traci.close()


# シミュレーションを開始する
def startSim():
    traci.start([sumoBinary, "-c", "../config/high-way.sumocfg"])
    print("Simulation started")


# 初期設定（車両の流入時間の設定）
def set_environment(inflow_0: int, inflow_1: int):
    global vehicle_instance
    global veh_id
    global departTime_lane0, departTime_lane1

    k_0 = int((simulation_time / 3600) * inflow_0)
    k_1 = int((simulation_time / 3600) * inflow_1)

    # 車両の流入時刻を決定
    departTime_lane0 = sorted(random.sample(range(int(simulation_time / 2.0)), k_0))
    departTime_lane1 = sorted(random.sample(range(int(simulation_time / 2.0)), k_1))

    # 2秒以上開けて流入
    departTime_lane0 = [round((n * 2.0), 1) + 0.1 for n in departTime_lane0]
    departTime_lane1 = [round((n * 2.0), 1) + 0.1 for n in departTime_lane1]

    print(departTime_lane0)
    print(departTime_lane1)


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
        )
    return True if sumo_time != simulation_time else False


def add_vehicle(alpha):
    global veh_id
    global departTime_lane0, departTime_lane1
    sumo_time = traci.simulation.getTime()

    alpha = float(alpha)

    # if sumo_time in departTime_lane0:
    if sumo_time in departTime_lane0:
        traci.vehicle.add(
            vehID=str(veh_id),
            routeID="r_pass",
            typeID="CAV_0",
            departLane="best",
            departPos="base",
            departSpeed="22",
        )
        instance = CAV(veh_id, alpha, withAgree=True)
        vehicle_instance.append(instance)
        veh_id += 1

    # if sumo_time in departTime_lane1:
    if sumo_time in departTime_lane1:
        traci.vehicle.add(
            vehID=str(veh_id),
            routeID="r_exit",
            typeID="CAV_1",
            departLane="best",
            departPos="base",
            departSpeed="22",
        )
        instance = CAV(veh_id, alpha, withAgree=True)
        vehicle_instance.append(instance)
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
    inflow_0 = int(args[3])  # 車両の流入数
    inflow_1 = int(args[4])  # 車両の流入数

    # this script has been called from the command line. It will start sumo as a server, then connect and run
    if options.nogui:
        sumoBinary = checkBinary("sumo")
    else:
        sumoBinary = checkBinary("sumo-gui")

    startSim()
    run(alpha, inflow_0, inflow_1)
