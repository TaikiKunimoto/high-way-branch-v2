import csv
import os
from datetime import datetime

time_step = 0.1  # [s]



class SimulationStatistics:
    # 急ブレーキの回数
    emergency_brake_count = 0

    def __init__(self, filename, output_dir="simulationStatistics/statistics"):
        self.output_dir = output_dir
        os.makedirs(output_dir, exist_ok=True)
        self.filename = self._create_filename(filename)
        self._create_csv_with_headers()

        self.total_travel_time = 0  # 全車両の走行時間
        self.r_pass_total_travel_time = 0  # r_pass車両の走行時間
        self.r_exit_total_travel_time = 0  # r_exit車両の走行時間

        self.vehcile_speed_data = []  # シミュレーションを完了した車輌の速度データ
        self.r_pass_vehicle_speed_data = []  # r_pass車輌の速度データ
        self.r_exit_vehicle_speed_data = []  # r_exit車輌の速度データ

        # Time-to-collision (TTC)
        self.TTC_THRESHOLD = 2.0  # TTCの閾値 [s]
        self.total_TET = 0  # Time Exposed TTC (TET) の累積値
        self.min_TTC = float("inf")  # 記録された最小TTC

    # 車輌ごとの travel time を計算
    def calculate_travel_time(self, route, departure_time, arrival_time):
        ins_travel_time = arrival_time - departure_time
        self.total_travel_time += ins_travel_time

        if route == "r_pass":
            self.r_pass_total_travel_time += ins_travel_time
        elif route == "r_exit":
            self.r_exit_total_travel_time += ins_travel_time

        return ins_travel_time

    # シミュレーション全体の平均 travel time を計算
    def _calculate_average_travel_time(self, total_travel_time, exit_vehicle):
        if len(exit_vehicle) == 0:
            return
        return total_travel_time / len(exit_vehicle)

    # 車輌ごとの average speed を計算
    def calculate_vehicle_average_spped(self, route, speed_history: list):
        if len(speed_history) == 0:
            return
        ins_average_speed = sum(speed_history) / len(speed_history)
        self.vehcile_speed_data.append(ins_average_speed)

        if route == "r_pass":
            self.r_pass_vehicle_speed_data.append(ins_average_speed)
        elif route == "r_exit":
            self.r_exit_vehicle_speed_data.append(ins_average_speed)

        return ins_average_speed

    # シミュレーション全体での平均 speed を計算
    def _calculate_average_speed(self, vehicle_speed_data):
        if len(vehicle_speed_data) == 0:
            return
        return sum(vehicle_speed_data) / len(vehicle_speed_data)

    # 公平性指標を計算
    def _calculate_fairness_index(self, results):
        overtake_count = 0
        overtaking_vehicles = set()
        departed_order = {
            veh_id: i for i, veh_id in enumerate(results["r_exit_departed_vehicle"])
        }
        # exit と running を結合
        conbined = [
            results["r_exit_exit_vehicle"],
            results["r_exit_running_vehicle"],
        ]
        conbined_order = {
            veh_id: i
            for i, veh_id in enumerate(
                [item for sublist in conbined for item in sublist]
            )
        }

        # 追い越しの回数を計算（シミュレーション中の車輌も考慮）
        for veh_id, dep_order in departed_order.items():
            if veh_id in conbined_order:
                current_order = conbined_order[veh_id]
                if current_order < dep_order:
                    overtake_count += dep_order - current_order
                    overtaking_vehicles.add(veh_id)

        fairness_index = (
            overtake_count / len(overtaking_vehicles) if overtake_count > 0 else 0
        )

        return overtake_count, len(overtaking_vehicles), fairness_index

    # Time-to-collision (TTC) を計算
    def calculate_TTC(self, distance, leader_speed, follower_speed):
        relative_speed = follower_speed - leader_speed
        if relative_speed <= 0:
            return None

        ttc = distance / relative_speed

        # 最小TTC値の更新
        if ttc < self.min_TTC:
            self.min_TTC = ttc

        # Time Exposed TTC (TET) の更新
        if ttc < self.TTC_THRESHOLD:
            self.total_TET += time_step

        return ttc

    # 急ブレーキの回数をカウント
    def increment_emergency_brake(self):
        SimulationStatistics.emergency_brake_count += 1

    def _create_filename(self, filename):
        timestamp = datetime.now().strftime("%Y%m%d_%H%M")
        return f"{self.output_dir}/{filename}_{timestamp}.csv"

    def _create_csv_with_headers(self):
        headers = [
            "simulation_time",
            "seed",
            "inflow_pass",
            "inflow_exit",
            "total_generated_vehicles",
            "total_departed_vehicles",
            "r_pass_departed_vehicles",
            "r_exit_departed_vehicles",
            "running_vehicles",
            "exited_vehicles",
            "canceled_vehicles",
            "traffic volume",
            "average_travel_time",
            "average_r_pass_travel_time",
            "average_r_exit_travel_time",
            "average_speed",
            "average_r_pass_speed",
            "average_r_exit_speed",
            "overtaking_count",
            "overtaiking_vehicle_count",
            "fairness_index",
            "min_TTC",
            "TET",
            "total_collisions",
            "total_vehicles_involved",
            "max_tail_position",
        ]
        with open(self.filename, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(headers)

    def add_result(self, simulation_time, seed, inflow_pass, inflow_exit, results):
        fairness_results = self._calculate_fairness_index(results)
        overtake_count = fairness_results[0] # 追い越し回数
        overtaking_vehicle_count = fairness_results[1] # 追い越し車輌数
        fairness_index = fairness_results[2] # 公平性指標
        row = [
            simulation_time,
            seed,
            inflow_pass,
            inflow_exit,
            results["total_generated_vehicle"],
            len(results["total_departed_vehicle"]),
            len(results["r_pass_departed_vehicle"]),
            len(results["r_exit_departed_vehicle"]),
            len(results["running_vehicle"]),
            len(results["exit_vehicle"]),
            len(results["canceled_vehicle"]),
            results["traffic_volume"],
            self._calculate_average_travel_time(
                self.total_travel_time, results["exit_vehicle"]
            ),  # 環境を出た車輌の平均走行時間
            self._calculate_average_travel_time(
                self.r_pass_total_travel_time, results["r_pass_exit_vehicle"]
            ),  # 環境を出たr_pass車輌の平均走行時間
            self._calculate_average_travel_time(
                self.r_exit_total_travel_time, results["r_exit_exit_vehicle"]
            ),  # 環境を出たr_exit車輌の平均走行時間
            self._calculate_average_speed(
                self.vehcile_speed_data
            ),  # 環境に残っている車輌を含めた平均速度
            self._calculate_average_speed(
                self.r_pass_vehicle_speed_data
            ),  # 環境に残っているr_pass車輌を含めた平均速度
            self._calculate_average_speed(
                self.r_exit_vehicle_speed_data
            ),  # 環境に残っているr_exit車輌を含めた平均速度
            overtake_count,
            overtaking_vehicle_count,
            fairness_index,
            self.min_TTC,
            self.total_TET,
            results["total_collisions"],
            results["total_vehicles_involved"],
            results["max_tail_position"],
        ]

        with open(self.filename, "a", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(row)
