import csv
import os
from datetime import datetime

time_step = 0.1  # [s]



class SimulationStatistics:
    # 急ブレーキの回数
    emergency_brake_count = 0

    def __init__(self, filename, output_dir="SimulationStatistics/statistics"):
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
        return total_travel_time / len(exit_vehicle)

    # 車輌ごとの average speed を計算
    def calculate_vehicle_average_spped(self, route, speed_history: list):
        ins_average_speed = sum(speed_history) / len(speed_history)
        self.vehcile_speed_data.append(ins_average_speed)

        if route == "r_pass":
            self.r_pass_vehicle_speed_data.append(ins_average_speed)
        elif route == "r_exit":
            self.r_exit_vehicle_speed_data.append(ins_average_speed)

        return ins_average_speed

    # シミュレーション全体での平均 speed を計算
    def _calculate_average_speed(self, vehicle_speed_data):
        return sum(vehicle_speed_data) / len(vehicle_speed_data)

    # 公平性指標を計算
    def _calculate_fairness_index(self, results):
        overtake_count = 0
        exit_order = {
            veh_id: i for i, veh_id in enumerate(results["r_exit_exit_vehicle"])
        }
        departed_order = {
            veh_id: i for i, veh_id in enumerate(results["r_exit_departed_vehicle"])
        }

        # 各車両について、出口を出た順番が入った順番より早い場合（追い越しが発生）
        for veh_id in results["r_exit_exit_vehicle"]:
            if exit_order[veh_id] < departed_order[veh_id]:
                # その車両が追い越した台数を計算（順番の差分）
                overtake_count += departed_order[veh_id] - exit_order[veh_id]

        return overtake_count

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
        return f"{self.output_dir}/simulation_results_{filename}_{timestamp}.csv"

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
            # "lane0_queue",
            # "lane1_queue",
            # "lane2_queue",
            "traffic volume",
            "average_travel_time",
            "average_r_pass_travel_time",
            "average_r_exit_travel_time",
            "average_speed",
            "average_r_pass_speed",
            "average_r_exit_speed",
            "fairness_index",
            # "emergency_brake_count",
            "min_TTC",
            "TET",
        ]
        with open(self.filename, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(headers)

    def add_result(self, simulation_time, seed, inflow_pass, inflow_exit, results):
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
            # len(results["lane0_queue"]),
            # len(results["lane1_queue"]),
            # len(results["lane2_queue"]),
            results["traffic_volume"],
            self._calculate_average_travel_time(
                self.total_travel_time, results["exit_vehicle"]
            ),
            self._calculate_average_travel_time(
                self.r_pass_total_travel_time, results["r_pass_exit_vehicle"]
            ),
            self._calculate_average_travel_time(
                self.r_exit_total_travel_time, results["r_exit_exit_vehicle"]
            ),
            self._calculate_average_speed(self.vehcile_speed_data),
            self._calculate_average_speed(self.r_pass_vehicle_speed_data),
            self._calculate_average_speed(self.r_exit_vehicle_speed_data),
            self._calculate_fairness_index(results),
            self.min_TTC,
            self.total_TET,
        ]

        with open(self.filename, "a", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(row)
