"""
Frenet optimal trajectory generator
author: Atsushi Sakai (@Atsushi_twi)
extend: KATO Sho
Ref:
- [Optimal Trajectory Generation for Dynamic Street Scenarios in a Frenet Frame]
(https://www.researchgate.net/profile/Moritz_Werling/publication/224156269_Optimal_Trajectory_Generation_for_Dynamic_Street_Scenarios_in_a_Frenet_Frame/links/54f749df0cf210398e9277af.pdf)
- [Optimal trajectory generation for dynamic street scenarios in a Frenet Frame]
(https://www.youtube.com/watch?v=Cj6tAQe7UCY)
"""

import copy
import math
import os
import sys
import time

import matplotlib.pyplot as plt
import numpy as np

try:
    from PathPlanning.CubicSpline import cubic_spline_planner
    from PathPlanning.QuinticPolynomialsPlanner.quintic_polynomials_planner import QuinticPolynomial
except ImportError:
    raise

# Parameter
MAX_SPEED = 16.67  # maximum speed [m/s]
MAX_ACCEL = 3.0  # maximum acceleration [m/ss]
MAX_DECEL = -5.0  # maximum acceleration [m/ss]
MAX_CURVATURE = 1.0  # maximum curvature [1/m]
tau = 1.5  # safe headway time [s]
D_0 = 5  # longitudinal safe margin [m]

CENTER_LINES = [-4.8, -8.0]  # y position of roads
LANE_WIDTH = 3.2  # [m]
timeStep = 0.1  # time tick [s]
minT = 1.0  # min prediction time [s]
maxT = 10.0  # max prediction time [s]
laneChangeDuration = 3.0  # [s]

D_T_S = 2.78  # (10 km/h) target speed sampling length [m/s]
N_S_SAMPLE = 5  # sampling number of target speed
DS = 8.0  # target position sampling length [m]
N_DS_SAMPLE = 5  # sampling number of target position

# cost weights
K_J = 0.1  # ジャークの大きさの重み
K_T = 1.0  # 収束時間の重み
K_D = 3.0  # 目標位置や目標速度との差分の重み
K_LAT = 1.0
K_LON = 1.0

ob = None


# 最終的な位置に興味がない場合
class QuarticPolynomial_follow:

    def __init__(self, xs, vxs, axs, vxe, axe, time):
        # calc coefficient of quartic polynomial

        self.a0 = xs
        self.a1 = vxs
        self.a2 = axs / 2.0

        A = np.array([[3 * time**2, 4 * time**3], [6 * time, 12 * time**2]])
        b = np.array([vxe - self.a1 - 2 * self.a2 * time, axe - 2 * self.a2])
        x = np.linalg.solve(A, b)

        self.a3 = x[0]
        self.a4 = x[1]

    def calc_point(self, t):
        xt = self.a0 + self.a1 * t + self.a2 * t**2 + self.a3 * t**3 + self.a4 * t**4

        return xt

    def calc_first_derivative(self, t):
        xt = self.a1 + 2 * self.a2 * t + 3 * self.a3 * t**2 + 4 * self.a4 * t**3

        return xt

    def calc_second_derivative(self, t):
        xt = 2 * self.a2 + 6 * self.a3 * t + 12 * self.a4 * t**2

        return xt

    def calc_third_derivative(self, t):
        xt = 6 * self.a3 + 24 * self.a4 * t

        return xt


class FrenetPath:

    def __init__(self):
        self.t = []
        self.d = []
        self.d_d = []
        self.d_dd = []
        self.d_ddd = []
        self.s = []
        self.s_d = []
        self.s_dd = []
        self.s_ddd = []
        self.cd = 0.0
        self.cv = 0.0
        self.cf = 0.0

        self.x = []
        self.y = []
        self.yaw = []
        self.ds = []
        self.c = []

        self.isLaneChange = False
        self.convergeTime = None
        self.pathID = ""
        self.type = ""
        self.vehID = ""
        self.vehLength = 5.0
        self.vehWidth = 1.8
        # 交渉に参加してほしい車両ID
        self.negotiationID = []
        self.yieldTo = None


# 最高速度を目指して走行
def calc_frenet_paths_free(
    c_s,
    c_s_d,
    c_s_dd,
    c_d,
    c_d_d,
    c_d_dd,
    targetSpeed=MAX_SPEED,
    center_lines=CENTER_LINES,
    duration=maxT,
):
    frenet_paths = []

    D_T_S = targetSpeed / 5
    tvlist = np.arange(0, targetSpeed + 0.01, D_T_S)

    # 収束時間が決まっている場合
    if duration:
        Ti = duration
        # 現在車線および隣接車線までの経路を計算
        for di in center_lines:
            # 横方向のモーションプランニング
            # 目的時間を変化させる
            fp = FrenetPath()
            fp.convergeTime = Ti

            # 五元方程式               始点　　　　　　　　終点
            lat_qp = QuinticPolynomial(c_d, c_d_d, c_d_dd, di, 0.0, 0.0, Ti)

            # 経路内のタイムステップ
            fp.t = [round(t, 3) for t in np.arange(0.0, Ti + 0.1, timeStep)]
            # 各タイムステップの横方向の位置、速度、加速度、躍度
            fp.d = [round(lat_qp.calc_point(t), 3) for t in fp.t]
            fp.d_d = [lat_qp.calc_first_derivative(t) for t in fp.t]
            fp.d_dd = [lat_qp.calc_second_derivative(t) for t in fp.t]
            fp.d_ddd = [lat_qp.calc_third_derivative(t) for t in fp.t]

            for tv in tvlist:
                # 縦方向のモーションプランニング (目標速度での走行を目指す場合)
                tfp = copy.deepcopy(fp)
                # 四元方程式               始点             終点
                lon_qp = QuarticPolynomial_follow(c_s, c_s_d, c_s_dd, tv, 0.0, Ti)

                # 各タイムステップの縦方向の位置、速度、加速度、躍度
                tfp.s = [round(lon_qp.calc_point(t), 3) for t in fp.t]
                tfp.s_d = [round(lon_qp.calc_first_derivative(t), 3) for t in fp.t]
                tfp.s_dd = [lon_qp.calc_second_derivative(t) for t in fp.t]
                tfp.s_ddd = [lon_qp.calc_third_derivative(t) for t in fp.t]

                Jp = sum(np.power(tfp.d_ddd, 2))  # square of jerk
                Js = sum(np.power(tfp.s_ddd, 2))  # square of jerk

                # 横方向のコスト　ジャークの自乗＋収束時間＋センターラインまでの距離自乗
                tfp.cd = K_J * Jp + K_T * Ti + K_D * tfp.d[-1] ** 2
                # 縦方向のコスト　ジャークの自乗＋収束速度＋目標スピードとの差分自乗
                tfp.cv = K_J * Js + K_T * Ti + K_D * ((MAX_SPEED - tfp.s_d[-1]) ** 2)
                # コスト関数（横方向コストと縦方向コストの和）
                tfp.cf = K_LAT * tfp.cd + K_LON * tfp.cv

                frenet_paths.append(tfp)

    # 収束時間が決まっていない場合
    else:
        for Ti in np.arange(3.0, maxT + 0.1, 1.0):
            for di in center_lines:
                # 横方向のモーションプランニング
                # 目的時間を変化させる
                fp = FrenetPath()
                fp.convergeTime = Ti

                # 五元方程式               始点　　　　　　　　終点
                lat_qp = QuinticPolynomial(c_d, c_d_d, c_d_dd, di, 0.0, 0.0, Ti)

                # 経路内のタイムステップ
                fp.t = [round(t, 3) for t in np.arange(0.0, Ti + 0.1, timeStep)]
                # 各タイムステップの横方向の位置、速度、加速度、躍度
                fp.d = [round(lat_qp.calc_point(t), 3) for t in fp.t]
                fp.d_d = [lat_qp.calc_first_derivative(t) for t in fp.t]
                fp.d_dd = [lat_qp.calc_second_derivative(t) for t in fp.t]
                fp.d_ddd = [lat_qp.calc_third_derivative(t) for t in fp.t]

                for tv in tvlist:
                    # 縦方向のモーションプランニング (目標速度での走行を目指す場合)
                    tfp = copy.deepcopy(fp)
                    # 四元方程式               始点             終点
                    lon_qp = QuarticPolynomial_follow(c_s, c_s_d, c_s_dd, tv, 0.0, Ti)

                    # 各タイムステップの縦方向の位置、速度、加速度、躍度
                    tfp.s = [round(lon_qp.calc_point(t), 3) for t in fp.t]
                    tfp.s_d = [round(lon_qp.calc_first_derivative(t), 3) for t in fp.t]
                    tfp.s_dd = [lon_qp.calc_second_derivative(t) for t in fp.t]
                    tfp.s_ddd = [lon_qp.calc_third_derivative(t) for t in fp.t]

                    Jp = sum(np.power(tfp.d_ddd, 2))  # square of jerk
                    Js = sum(np.power(tfp.s_ddd, 2))  # square of jerk

                    # 横方向のコスト　ジャークの自乗＋収束時間＋センターラインまでの距離自乗
                    tfp.cd = K_J * Jp + K_T * Ti + K_D * tfp.d[-1] ** 2
                    # 縦方向のコスト　ジャークの自乗＋収束速度＋目標スピードとの差分自乗
                    tfp.cv = (
                        K_J * Js + K_T * Ti + K_D * ((MAX_SPEED - tfp.s_d[-1]) ** 2)
                    )
                    # コスト関数（横方向コストと縦方向コストの和）
                    tfp.cf = K_LAT * tfp.cd + K_LON * tfp.cv

                    frenet_paths.append(tfp)

    return frenet_paths


def calc_frenet_paths_following(
    c_s, c_s_d, c_s_dd, c_d, c_d_d, c_d_dd, target_pos, target_speed, duration
):
    frenet_paths = []

    di = c_d
    Ti = duration

    # 横方向のモーションプランニング
    fp = FrenetPath()
    fp.convergeTime = Ti

    # 五元方程式              始点　　　　　　　　終点
    lat_qp = QuinticPolynomial(c_d, c_d_d, c_d_dd, di, 0.0, 0.0, Ti)

    # 経路内のタイムステップ
    fp.t = [round(t, 3) for t in np.arange(0.0, Ti + 0.1, timeStep)]
    # 各タイムステップの横方向の位置、速度、加速度、躍度
    fp.d = [round(lat_qp.calc_point(t), 3) for t in fp.t]
    fp.d_d = [lat_qp.calc_first_derivative(t) for t in fp.t]
    fp.d_dd = [lat_qp.calc_second_derivative(t) for t in fp.t]
    fp.d_ddd = [lat_qp.calc_third_derivative(t) for t in fp.t]

    # 縦方向のモーションプランニング (目標速度での走行を目指す場合)
    # 目的速度と目的地点を変化させる
    D_P = 10.0  # [m]
    N_P_SAMPLE = 10

    for tp in np.arange(target_pos, max(target_pos - D_P * N_P_SAMPLE, c_s), -1 * D_P):

        # tp = target_pos
        tfp = copy.deepcopy(fp)

        # 五元方程式               始点　　　　　　　　終点
        lon_qp = QuinticPolynomial(c_s, c_s_d, c_s_dd, tp, target_speed, 0.0, Ti)

        # 各タイムステップの縦方向の位置、速度、加速度、躍度
        tfp.s = [round(lon_qp.calc_point(t), 3) for t in fp.t]
        tfp.s_d = [round(lon_qp.calc_first_derivative(t), 3) for t in fp.t]
        tfp.s_dd = [lon_qp.calc_second_derivative(t) for t in fp.t]
        tfp.s_ddd = [lon_qp.calc_third_derivative(t) for t in fp.t]

        Jp = sum(np.power(tfp.d_ddd, 2))  # square of jerk
        Js = sum(np.power(tfp.s_ddd, 2))  # square of jerk

        # 横方向のコスト　ジャークの自乗＋収束時間＋センターラインまでの距離自乗
        tfp.cd = K_J * Jp + K_T * Ti + K_D * tfp.d[-1] ** 2
        # 縦方向のコスト　ジャークの自乗＋収束速度＋目的地点との差分自乗
        tfp.cv = K_J * Js + K_T * Ti + K_D * ((target_pos - tfp.s[-1]) ** 2)
        # コスト関数（横方向コストと縦方向コストの和）
        tfp.cf = K_LAT * tfp.cd + K_LON * tfp.cv

        frenet_paths.append(tfp)

    return frenet_paths


def calc_frenet_paths_yield(
    c_s, c_s_d, c_s_dd, c_d, c_d_d, c_d_dd, target_pos, target_speed, duration
):
    frenet_paths = []
    target_acc = 0

    di = c_d
    Ti = duration

    for Ti in np.arange(duration + tau, duration + 10 + 0.1, 1.0):
        # 横方向のモーションプランニング
        fp = FrenetPath()
        fp.convergeTime = Ti

        # 五元方程式              始点　　　　　　　　終点
        lat_qp = QuinticPolynomial(c_d, c_d_d, c_d_dd, di, 0.0, 0.0, Ti)

        # 経路内のタイムステップ
        fp.t = [round(t, 3) for t in np.arange(0.0, Ti + 0.1, timeStep)]
        # 各タイムステップの横方向の位置、速度、加速度、躍度
        fp.d = [round(lat_qp.calc_point(t), 3) for t in fp.t]
        fp.d_d = [lat_qp.calc_first_derivative(t) for t in fp.t]
        fp.d_dd = [lat_qp.calc_second_derivative(t) for t in fp.t]
        fp.d_ddd = [lat_qp.calc_third_derivative(t) for t in fp.t]

        # 縦方向のモーションプランニング (目標速度での走行を目指す場合)
        # 目的速度を変化させる
        D_T_S = MAX_SPEED / 5
        tvlist = np.arange(0, MAX_SPEED + 0.01, D_T_S)

        tp = target_pos
        for tv in tvlist:
            tfp = copy.deepcopy(fp)

            # 五元方程式               始点　　　　　　　　終点
            lon_qp = QuinticPolynomial(c_s, c_s_d, c_s_dd, tp, tv, target_acc, Ti)

            # 各タイムステップの縦方向の位置、速度、加速度、躍度
            tfp.s = [round(lon_qp.calc_point(t), 3) for t in fp.t]
            tfp.s_d = [round(lon_qp.calc_first_derivative(t), 3) for t in fp.t]
            tfp.s_dd = [lon_qp.calc_second_derivative(t) for t in fp.t]
            tfp.s_ddd = [lon_qp.calc_third_derivative(t) for t in fp.t]

            Jp = sum(np.power(tfp.d_ddd, 2))  # square of jerk
            Js = sum(np.power(tfp.s_ddd, 2))  # square of jerk

            # 横方向のコスト　ジャークの自乗＋収束時間＋センターラインまでの距離自乗
            tfp.cd = K_J * Jp + K_T * Ti + K_D * tfp.d[-1] ** 2
            # 縦方向のコスト　ジャークの自乗＋収束速度＋目的速度との差分自乗
            tfp.cv = K_J * Js + K_T * Ti + K_D * ((target_speed - tfp.s_d[-1]) ** 2)
            # コスト関数（横方向コストと縦方向コストの和）
            tfp.cf = K_LAT * tfp.cd + K_LON * tfp.cv

            frenet_paths.append(tfp)

    return frenet_paths


def calc_frenet_paths_stop(
    c_s, c_s_d, c_s_dd, c_d, c_d_d, c_d_dd, target_pos, center_lines
):
    frenet_paths = []
    target_speed = 0
    target_acc = 0

    for di in center_lines:
        for Ti in np.arange(minT, maxT + 0.1, 1.0):
            # 横方向のモーションプランニング
            fp = FrenetPath()
            fp.convergeTime = Ti

            # 3秒間かけて横方向の移動（車線変更）を行う
            # 五元方程式              始点　　　　　　　　終点
            lat_qp = QuinticPolynomial(c_d, c_d_d, c_d_dd, di, 0.0, 0.0, Ti)

            # 経路内のタイムステップ
            fp.t = [round(t, 3) for t in np.arange(0.0, Ti + 0.1, timeStep)]
            # 各タイムステップの横方向の位置、速度、加速度、躍度
            fp.d = [round(lat_qp.calc_point(t), 3) for t in fp.t]
            fp.d_d = [lat_qp.calc_first_derivative(t) for t in fp.t]
            fp.d_dd = [lat_qp.calc_second_derivative(t) for t in fp.t]
            fp.d_ddd = [lat_qp.calc_third_derivative(t) for t in fp.t]

            # 縦方向のモーションプランニング (目標速度での走行を目指す場合)
            tfp = copy.deepcopy(fp)

            # 五元方程式               始点　　　　　　　　終点
            lon_qp = QuinticPolynomial(
                c_s, c_s_d, c_s_dd, target_pos, target_speed, target_acc, Ti
            )

            # 各タイムステップの縦方向の位置、速度、加速度、躍度
            tfp.s = [round(lon_qp.calc_point(t), 3) for t in fp.t]
            tfp.s_d = [round(lon_qp.calc_first_derivative(t), 3) for t in fp.t]
            tfp.s_dd = [round(lon_qp.calc_second_derivative(t), 3) for t in fp.t]
            tfp.s_ddd = [lon_qp.calc_third_derivative(t) for t in fp.t]

            Jp = sum(np.power(tfp.d_ddd, 2))  # square of jerk
            Js = sum(np.power(tfp.s_ddd, 2))  # square of jerk

            # 横方向のコスト　ジャークの自乗＋収束時間＋センターラインまでの距離自乗
            tfp.cd = K_J * Jp + K_T * Ti + K_D * tfp.d[-1] ** 2
            # 縦方向のコスト　ジャークの自乗＋収束速度＋目的地点との差分自乗
            tfp.cv = K_J * Js + K_T * Ti + K_D * ((target_pos - tfp.s[-1]) ** 2)
            # コスト関数（横方向コストと縦方向コストの和）
            tfp.cf = K_LAT * tfp.cd + K_LON * tfp.cv

            frenet_paths.append(tfp)

    return frenet_paths


def calc_frenet_paths_desired(
    c_s, c_s_d, c_s_dd, c_d, c_d_d, c_d_dd, duration=maxT, center_lines=CENTER_LINES
):
    frenet_paths = []
    Ti = duration

    # 現在車線および隣接車線までの経路を計算
    for di in center_lines:
        # 横方向のモーションプランニング
        # 目的時間を変化させる
        fp = FrenetPath()
        fp.convergeTime = Ti

        # 五元方程式               始点　　　　　　　　終点
        lat_qp = QuinticPolynomial(c_d, c_d_d, c_d_dd, di, 0.0, 0.0, Ti)

        # 経路内のタイムステップ
        fp.t = [round(t, 3) for t in np.arange(0.0, Ti + 0.1, timeStep)]
        # 各タイムステップの横方向の位置、速度、加速度、躍度
        fp.d = [round(lat_qp.calc_point(t), 3) for t in fp.t]
        fp.d_d = [lat_qp.calc_first_derivative(t) for t in fp.t]
        fp.d_dd = [lat_qp.calc_second_derivative(t) for t in fp.t]
        fp.d_ddd = [lat_qp.calc_third_derivative(t) for t in fp.t]

        # 縦方向のモーションプランニング (目標速度での走行を目指す場合)
        tfp = copy.deepcopy(fp)
        # 四元方程式               始点             終点
        lon_qp = QuarticPolynomial_follow(c_s, c_s_d, c_s_dd, MAX_SPEED, 0.0, Ti)

        # 各タイムステップの縦方向の位置、速度、加速度、躍度
        tfp.s = [round(lon_qp.calc_point(t), 3) for t in fp.t]
        tfp.s_d = [round(lon_qp.calc_first_derivative(t), 3) for t in fp.t]
        tfp.s_dd = [lon_qp.calc_second_derivative(t) for t in fp.t]
        tfp.s_ddd = [lon_qp.calc_third_derivative(t) for t in fp.t]

        Jp = sum(np.power(tfp.d_ddd, 2))  # square of jerk
        Js = sum(np.power(tfp.s_ddd, 2))  # square of jerk

        # 横方向のコスト　ジャークの自乗＋収束時間＋センターラインまでの距離自乗
        tfp.cd = K_J * Jp + K_T * Ti + K_D * tfp.d[-1] ** 2
        # 縦方向のコスト　ジャークの自乗＋収束速度＋目標スピードとの差分自乗
        tfp.cv = K_J * Js + K_T * Ti + K_D * ((MAX_SPEED - tfp.s_d[-1]) ** 2)
        # コスト関数（横方向コストと縦方向コストの和）
        tfp.cf = K_LAT * tfp.cd + K_LON * tfp.cv

        frenet_paths.append(tfp)

    return frenet_paths


def emergency_stop(pos_x, speed, pos_y, route):

    print("Emergency Stop")
    frenet_paths = []
    # 横方向のモーションプランニング
    fp = FrenetPath()
    fp.convergeTime = 10.0
    Ti = 10.0
    # 経路内のタイムステップ
    fp.t = [round(t, 3) for t in np.arange(0.0, Ti + 0.1, timeStep)]
    # 各タイムステップの横方向の位置、速度、加速度、躍度
    fp.d_d = [0 for t in fp.t]
    fp.d_dd = [0 for t in fp.t]
    fp.d_ddd = [0 for t in fp.t]

    # 縦方向のモーションプランニング (目標速度での走行を目指す場合)
    tfp = copy.deepcopy(fp)

    count = math.ceil(speed / abs(MAX_DECEL) / timeStep)
    xmax = (
        pos_x
        + speed * (count * timeStep)
        + (1 / 2) * MAX_DECEL * ((count * timeStep) ** 2)
    )
    tfp.x = [
        pos_x + speed * t * timeStep + (1 / 2) * MAX_DECEL * ((t * timeStep) ** 2)
        for t in range(count)
    ]
    tfp.x.extend([xmax for i in range(len(fp.t) - count)])
    tfp.y = [pos_y for t in tfp.t]

    tfp.s_d = [max(speed + MAX_DECEL * t, 0) for t in fp.t]
    tfp.s_dd = [MAX_DECEL for t in range(count)]
    tfp.s_dd.extend([0.0 for i in range(len(fp.t) - count)])
    tfp.s_ddd = [0 for t in fp.t]

    frenet_paths.append(tfp)

    return frenet_paths


def calc_global_paths(fplist, csp):
    for fp in fplist:
        # calc global positions
        for i in range(len(fp.s)):
            ix, iy = csp.calc_position(fp.s[i])
            if ix is None:
                break
            i_yaw = csp.calc_yaw(fp.s[i])
            di = fp.d[i]
            fx = ix + di * math.cos(i_yaw + math.pi / 2.0)
            fy = iy + di * math.sin(i_yaw + math.pi / 2.0)
            fp.x.append(fx)
            fp.y.append(fy)

        # calc yaw and ds
        for i in range(len(fp.x) - 1):
            dx = fp.x[i + 1] - fp.x[i]
            dy = fp.y[i + 1] - fp.y[i]
            fp.yaw.append(math.atan2(dy, dx))
            dstmp = math.hypot(dx, dy)
            if dstmp == 0:
                dstmp = 0.0001
            fp.ds.append(dstmp)

        fp.yaw.append(fp.yaw[-1])
        fp.ds.append(fp.ds[-1])

        # calc curvature
        for i in range(len(fp.yaw) - 1):
            fp.c.append((fp.yaw[i + 1] - fp.yaw[i]) / fp.ds[i])

    return fplist


def check_collision(fp, ob):
    for i in range(len(ob[:, 0])):
        s = [abs(ix - ob[i, 0]) for ix in fp.s]
        d = [abs(iy - ob[i, 1]) for iy in fp.d]

        # 車両を矩形として衝突判定
        for j in range(len(s)):
            if s[j] <= fp.vehLength + D_0 and d[j] <= fp.vehWidth:
                return False

    return True


def check_paths(fplist, ob, route=None, max_speed=MAX_SPEED, targetPos=None):
    ok_ind = []
    for i, _ in enumerate(fplist):
        if len(fplist[i].t) != len(fplist[i].x):
            continue

        if any(
            [v > max_speed + 0.1 for v in fplist[i].s_d]
        ):  # 最大速度を超えないかチェック
            continue
        if any([v < -0.1 for v in fplist[i].s_d]):  # 速度がマイナスにならないかチェック
            continue
        if any(
            [a > MAX_ACCEL for a in fplist[i].s_dd]
        ):  # 最大加速度を超えないかチェック
            continue
        if any(
            [a < MAX_DECEL for a in fplist[i].s_dd]
        ):  # 最大減速度を超えないかチェック
            continue
        if not check_collision(fplist[i], ob):  # 障害物との衝突判定
            # print("collide object")
            continue

        ok_ind.append(i)

    return [fplist[i] for i in ok_ind]


def generate_target_course(x, y):
    csp = cubic_spline_planner.Spline2D(x, y)
    s = np.arange(0, csp.s[-1], 0.1)

    rx, ry, ryaw, rk = [], [], [], []
    for i_s in s:
        ix, iy = csp.calc_position(i_s)
        rx.append(ix)
        ry.append(iy)
        ryaw.append(csp.calc_yaw(i_s))
        rk.append(csp.calc_curvature(i_s))

    return rx, ry, ryaw, rk, csp


# maxT秒に満たない経路は等速で動くと仮定してmaxT秒分まで補間する
def interpolatePath(fplist):

    for path in fplist:
        if path.convergeTime:
            extendStep = round(maxT / timeStep - len(path.t) + 1)
            if extendStep > 0:
                # 収束以降は等速で移動すると仮定
                lastSpeed = path.s_d[-1]
                lastPosS = path.s[-1]
                path.t.extend(
                    [
                        round(path.t[-1] + timeStep * (i + 1), 3)
                        for i in range(extendStep)
                    ]
                )
                path.d.extend([path.d[-1] for n in range(extendStep)])
                path.d_d.extend([0.0 for n in range(extendStep)])
                path.d_dd.extend([0.0 for n in range(extendStep)])
                path.d_ddd.extend([0.0 for n in range(extendStep)])
                path.s.extend(
                    [
                        lastPosS + lastSpeed * (n + 1) * timeStep
                        for n in range(extendStep)
                    ]
                )
                path.s_d.extend([lastSpeed for n in range(extendStep)])
                path.s_dd.extend([0.0 for n in range(extendStep)])
                path.s_ddd.extend([0.0 for n in range(extendStep)])

    return fplist


def plot_path(fplist, ob):

    plt.figure(figsize=(8, 4))
    for path in fplist:
        plt.title("longitudinal position")
        plt.plot(path.t[0:], path.s[0:])
        # plt.plot(ob[:, 0], ob[:, 1], "xk", markersize=10)

    plt.figure(figsize=(8, 4))
    for path in fplist:
        plt.title("longitudinal velocity")
        plt.plot(path.t[0:], path.s_d[0:])
        plt.hlines([MAX_SPEED], 0, 10, "red", linestyles="dashed")

    plt.figure(figsize=(8, 4))
    for path in fplist:
        plt.title("longitudinal acceleration")
        plt.plot(path.t[0:], path.s_dd[0:])
        plt.hlines([MAX_ACCEL], 0, 10, "red", linestyles="dashed")
        plt.hlines([MAX_DECEL], 0, 10, "red", linestyles="dashed")

    # plt.figure(figsize=(8,4))
    # for path in fplist:
    #     plt.title("lateral velocity")
    #     plt.plot(path.t[0:], path.d_d[0:])

    # plt.figure(figsize=(8,4))
    # for path in fplist:
    #     plt.title("lateral acceleration")
    #     plt.plot(path.t[0:], path.d_dd[0:])

    plt.show()


def compare_path(path1, path2):

    plt.figure(figsize=(8, 4))
    plt.title("position")
    plt.plot(path1.x[0:], path1.y[0:], color="blue", label=path1.pathID)
    plt.plot(
        path1.x[::10],
        path1.y[::10],
        color="blue",
        marker=".",
        markersize=10,
        linewidth=0,
    )
    plt.plot(path2.x[0:], path2.y[0:], color="orange", label=path2.pathID)
    plt.plot(
        path2.x[::10],
        path2.y[::10],
        color="orange",
        marker=".",
        markersize=10,
        linewidth=0,
    )
    plt.legend()
    plt.show()


def generate_frenet_frame(
    objects,
    c_s,
    c_s_d,
    c_s_dd,
    c_d,
    mode,
    route,
    target_speed=MAX_SPEED,
    c_d_d=0,
    c_d_dd=0,
    target_pos=0,
    target_acc=0.0,
    duration=None,
    center_lines=CENTER_LINES,
    simTime=0.0,
    vehInstance=None,
):

    print(
        "mode",
        mode,
        "route",
        route,
        "duration",
        duration,
        ":c_s",
        c_s,
        ":c_s_d",
        c_s_d,
        ":c_s_dd",
        c_s_dd,
        "target_pos",
        "{:.5g}".format(target_pos),
        "target_spped",
        target_speed,
        "time",
        simTime,
    )

    global ob
    ob = objects
    wx = [n * 100 for n in range(8)]
    wy = [0 for n in range(8)]
    _, _, _, _, csp = generate_target_course(wx, wy)

    if mode == "free":
        fplist = calc_frenet_paths_free(
            c_s,
            c_s_d,
            c_s_dd,
            c_d,
            c_d_d,
            c_d_dd,
            center_lines=center_lines,
            duration=duration,
        )
        fplist = interpolatePath(fplist)
        fplist = calc_global_paths(fplist, csp)
        fplist = check_paths(fplist, ob, route=route, targetPos=target_pos)

    elif mode == "follow":
        fplist = calc_frenet_paths_following(
            c_s,
            c_s_d,
            c_s_dd,
            c_d,
            c_d_d,
            c_d_dd,
            target_pos,
            target_speed=target_speed,
            duration=duration,
        )
        fplist = interpolatePath(fplist)
        fplist = calc_global_paths(fplist, csp)
        fplist = check_paths(fplist, ob, route=route, targetPos=target_pos)

    elif mode == "desired":
        fplist = calc_frenet_paths_desired(
            c_s,
            c_s_d,
            c_s_dd,
            c_d,
            c_d_d,
            c_d_dd,
            duration=duration,
            center_lines=center_lines,
        )
        fplist = interpolatePath(fplist)
        fplist = calc_global_paths(fplist, csp)
        fplist = check_paths(fplist, ob, route=route, targetPos=target_pos)

    elif mode == "yield":
        fplist = calc_frenet_paths_yield(
            c_s,
            c_s_d,
            min(c_s_dd, 0.0),
            c_d,
            c_d_d,
            c_d_dd,
            target_pos,
            target_speed=target_speed,
            duration=duration,
        )
        fplist = interpolatePath(fplist)
        fplist = calc_global_paths(fplist, csp)
        fplist = check_paths(fplist, ob, route=route, targetPos=target_pos)

    elif mode == "stop":
        fplist = calc_frenet_paths_stop(
            c_s,
            c_s_d,
            min(c_s_dd, 0.0),
            c_d,
            c_d_d,
            c_d_dd,
            target_pos,
            center_lines=center_lines,
        )
        fplist = interpolatePath(fplist)
        fplist = calc_global_paths(fplist, csp)
        fplist = check_paths(fplist, ob, route=route, targetPos=target_pos)

    # 経路中の時間をシミュレーション時間に一致させる
    for path in fplist:
        path.vehID = vehInstance.id
        path.t = [round(n + simTime, 1) for n in path.t]

    return fplist
