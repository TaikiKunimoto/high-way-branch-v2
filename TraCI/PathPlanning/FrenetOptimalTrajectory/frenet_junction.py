"""
Frenet optimal trajectory generator
author: Atsushi Sakai (@Atsushi_twi)
Ref:
- [Optimal Trajectory Generation for Dynamic Street Scenarios in a Frenet Frame]
(https://www.researchgate.net/profile/Moritz_Werling/publication/224156269_Optimal_Trajectory_Generation_for_Dynamic_Street_Scenarios_in_a_Frenet_Frame/links/54f749df0cf210398e9277af.pdf)
- [Optimal trajectory generation for dynamic street scenarios in a Frenet Frame]
(https://www.youtube.com/watch?v=Cj6tAQe7UCY)
"""

import numpy as np
import matplotlib.pyplot as plt
import copy
import math
import sys
import os
import time

sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../QuinticPolynomialsPlanner/")
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../CubicSpline/")

try:
    from quintic_polynomials_planner import QuinticPolynomial
    import cubic_spline_planner
except ImportError:
    raise

# Parameter
MAX_SPEED = 11.11  # maximum speed [m/s]
MAX_SPEED_INTERSECTION = 5.56  # maximum speed [m/s]
MAX_ACCEL = 3.0  # maximum acceleration [m/ss]
MAX_DECEL = -5.0  # maximum acceleration [m/ss]
MAX_CURVATURE = 1.0  # maximum curvature [1/m]
tau = 1.5 # safe headway time [s]
D_0 = 5 # longitudinal safe margin [m]

LANE_WIDTH = 3.2 # [m]
timeStep = 0.1  # time tick [s]
minT = 1.0  # min prediction time [s]
maxT = 10.0 # max prediction time [s]
laneChangeDuration = 3.0 # [s]

D_T_S = 2.78 #(10 km/h) target speed sampling length [m/s]
N_S_SAMPLE = 5  # sampling number of target speed
DS = 8.0 # target position sampling length [m]
N_DS_SAMPLE = 5  # sampling number of target position

# cost weights
K_J = 0.1 # ジャークの大きさの重み
K_T = 1.0 # 収束時間の重み
K_D = 3.0 # 目標位置や目標速度との差分の重み
K_LAT = 1.0
K_LON = 1.0

ob = None


# 最終的な位置に興味がない場合
class QuarticPolynomial:

    def __init__(self, xs, vxs, axs, vxe, axe, time):
        # calc coefficient of quartic polynomial

        self.a0 = xs
        self.a1 = vxs
        self.a2 = axs / 2.0

        A = np.array([[3 * time ** 2, 4 * time ** 3],
                      [6 * time, 12 * time ** 2]])
        b = np.array([vxe - self.a1 - 2 * self.a2 * time,
                      axe - 2 * self.a2])
        x = np.linalg.solve(A, b)

        self.a3 = x[0]
        self.a4 = x[1]

    def calc_point(self, t):
        xt = self.a0 + self.a1 * t + self.a2 * t ** 2 + \
             self.a3 * t ** 3 + self.a4 * t ** 4

        return xt

    def calc_first_derivative(self, t):
        xt = self.a1 + 2 * self.a2 * t + \
             3 * self.a3 * t ** 2 + 4 * self.a4 * t ** 3

        return xt

    def calc_second_derivative(self, t):
        xt = 2 * self.a2 + 6 * self.a3 * t + 12 * self.a4 * t ** 2

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
        self.isTurn = False
        self.convergeTime = None
        self.pathID = ''
        self.type = ''
        self.vehID = ''
        self.vehLength = 5.0
        self.vehWidth = 1.8
        # 交渉に参加してほしい車両ID
        self.negotiationID = []
        self.yieldTo = None


# 最高速度を目指して走行
def calc_frenet_paths_free(c_s, c_s_d, c_s_dd, c_d, c_d_d, c_d_dd, targetSpeed=MAX_SPEED, duration=None):
    frenet_paths = []

    D_T_S = targetSpeed/5
    tvlist = np.arange(0, targetSpeed+0.01, D_T_S)
    # 収束時間が決まっている場合
    if duration:
        Ti = duration
        #横方向のモーションプランニング
        #目的時間を変化させる
        fp = FrenetPath()
        fp.convergeTime = Ti

        #五元方程式               始点　　　　　　　　終点　　　
        lat_qp = QuinticPolynomial(c_d, c_d_d, c_d_dd, c_d, 0.0, 0.0, Ti)

        #経路内のタイムステップ
        fp.t = [round(t, 3) for t in np.arange(0.0, Ti+0.1, timeStep)]
        #各タイムステップの横方向の位置、速度、加速度、躍度
        fp.d = [round(lat_qp.calc_point(t), 3) for t in fp.t]
        fp.d_d = [lat_qp.calc_first_derivative(t) for t in fp.t]
        fp.d_dd = [lat_qp.calc_second_derivative(t) for t in fp.t]
        fp.d_ddd = [lat_qp.calc_third_derivative(t) for t in fp.t]


        for tv in tvlist:
            #縦方向のモーションプランニング (目標速度での走行を目指す場合)
            tfp = copy.deepcopy(fp)
            #四元方程式               始点             終点
            lon_qp = QuarticPolynomial(c_s, c_s_d, c_s_dd, tv, 0.0, Ti)

            #各タイムステップの縦方向の位置、速度、加速度、躍度
            tfp.s = [round(lon_qp.calc_point(t), 3) for t in fp.t]
            tfp.s_d = [round(lon_qp.calc_first_derivative(t), 3) for t in fp.t]
            tfp.s_dd = [lon_qp.calc_second_derivative(t) for t in fp.t]
            tfp.s_ddd = [lon_qp.calc_third_derivative(t) for t in fp.t]

            Jp = sum(np.power(tfp.d_ddd, 2))  # square of jerk
            Js = sum(np.power(tfp.s_ddd, 2))  # square of jerk

            #横方向のコスト　ジャークの自乗＋収束時間＋センターラインまでの距離自乗
            tfp.cd = K_J * Jp + K_T * Ti + K_D * tfp.d[-1] ** 2
            #縦方向のコスト　ジャークの自乗＋収束速度＋目標スピードとの差分自乗
            tfp.cv = K_J * Js + K_T * Ti + K_D * ((MAX_SPEED - tfp.s_d[-1]) ** 2)
            #コスト関数（横方向コストと縦方向コストの和）
            tfp.cf = K_LAT * tfp.cd + K_LON * tfp.cv

            frenet_paths.append(tfp)

    # 収束時間が決まっていない場合
    else:
        for Ti in np.arange(3.0, maxT+0.1, 1.0):
            #横方向のモーションプランニング
            #目的時間を変化させる
            fp = FrenetPath()
            fp.convergeTime = Ti

            #五元方程式               始点　　　　　　　　終点　　　
            lat_qp = QuinticPolynomial(c_d, c_d_d, c_d_dd, c_d, 0.0, 0.0, Ti)

            #経路内のタイムステップ
            fp.t = [round(t, 3) for t in np.arange(0.0, Ti+0.1, timeStep)]
            #各タイムステップの横方向の位置、速度、加速度、躍度
            fp.d = [round(lat_qp.calc_point(t), 3) for t in fp.t]
            fp.d_d = [lat_qp.calc_first_derivative(t) for t in fp.t]
            fp.d_dd = [lat_qp.calc_second_derivative(t) for t in fp.t]
            fp.d_ddd = [lat_qp.calc_third_derivative(t) for t in fp.t]

            for tv in tvlist:
                #縦方向のモーションプランニング (目標速度での走行を目指す場合)
                tfp = copy.deepcopy(fp)
                #四元方程式               始点             終点
                lon_qp = QuarticPolynomial(c_s, c_s_d, c_s_dd, tv, 0.0, Ti)

                #各タイムステップの縦方向の位置、速度、加速度、躍度
                tfp.s = [round(lon_qp.calc_point(t), 3) for t in fp.t]
                tfp.s_d = [round(lon_qp.calc_first_derivative(t), 3) for t in fp.t]
                tfp.s_dd = [lon_qp.calc_second_derivative(t) for t in fp.t]
                tfp.s_ddd = [lon_qp.calc_third_derivative(t) for t in fp.t]

                Jp = sum(np.power(tfp.d_ddd, 2))  # square of jerk
                Js = sum(np.power(tfp.s_ddd, 2))  # square of jerk

                #横方向のコスト　ジャークの自乗＋収束時間＋センターラインまでの距離自乗
                tfp.cd = K_J * Jp + K_T * Ti + K_D * tfp.d[-1] ** 2
                #縦方向のコスト　ジャークの自乗＋収束速度＋目標スピードとの差分自乗
                tfp.cv = K_J * Js + K_T * Ti + K_D * ((MAX_SPEED - tfp.s_d[-1]) ** 2)
                #コスト関数（横方向コストと縦方向コストの和）
                tfp.cf = K_LAT * tfp.cd + K_LON * tfp.cv

                frenet_paths.append(tfp)            

    return frenet_paths


def calc_frenet_paths_following(c_s, c_s_d, c_s_dd, c_d, c_d_d, c_d_dd, target_pos, target_speed, duration):
    frenet_paths = []

    di = c_d
    Ti = duration

    #横方向のモーションプランニング
    fp = FrenetPath()
    fp.convergeTime = Ti

    # 五元方程式              始点　　　　　　　　終点　　　
    lat_qp = QuinticPolynomial(c_d, c_d_d, c_d_dd, di, 0.0, 0.0, Ti)

    #経路内のタイムステップ
    fp.t = [round(t, 3) for t in np.arange(0.0, Ti+0.1, timeStep)]
    #各タイムステップの横方向の位置、速度、加速度、躍度
    fp.d = [round(lat_qp.calc_point(t), 3) for t in fp.t]
    fp.d_d = [lat_qp.calc_first_derivative(t) for t in fp.t]
    fp.d_dd = [lat_qp.calc_second_derivative(t) for t in fp.t]
    fp.d_ddd = [lat_qp.calc_third_derivative(t) for t in fp.t]

    #縦方向のモーションプランニング (目標速度での走行を目指す場合)
    #目的速度と目的地点を変化させる
    D_P = 10.0 # [m]
    N_P_SAMPLE = 10
    
    for tp in np.arange(target_pos, max(target_pos - D_P*N_P_SAMPLE, c_s), -1*D_P):

    # tp = target_pos
        tfp = copy.deepcopy(fp)

        #五元方程式               始点　　　　　　　　終点　　　
        lon_qp = QuinticPolynomial(c_s, c_s_d, c_s_dd, tp, target_speed, 0.0, Ti)

        #各タイムステップの縦方向の位置、速度、加速度、躍度
        tfp.s = [round(lon_qp.calc_point(t), 3) for t in fp.t]
        tfp.s_d = [round(lon_qp.calc_first_derivative(t), 3) for t in fp.t]
        tfp.s_dd = [lon_qp.calc_second_derivative(t) for t in fp.t]
        tfp.s_ddd = [lon_qp.calc_third_derivative(t) for t in fp.t]

        Jp = sum(np.power(tfp.d_ddd, 2))  # square of jerk
        Js = sum(np.power(tfp.s_ddd, 2))  # square of jerk

        #横方向のコスト　ジャークの自乗＋収束時間＋センターラインまでの距離自乗
        tfp.cd = K_J * Jp + K_T * Ti + K_D * tfp.d[-1] ** 2
        #縦方向のコスト　ジャークの自乗＋収束速度＋目的地点との差分自乗
        tfp.cv = K_J * Js + K_T * Ti + K_D * ((target_pos - tfp.s[-1]) ** 2)
        #コスト関数（横方向コストと縦方向コストの和）
        tfp.cf = K_LAT * tfp.cd + K_LON * tfp.cv

        frenet_paths.append(tfp)

    return frenet_paths


def calc_frenet_paths_yield(c_s, c_s_d, c_s_dd, c_d, c_d_d, c_d_dd, target_pos, target_speed, duration):
    frenet_paths = []
    target_acc = 0

    di = c_d
    Ti = duration

    for Ti in np.arange(duration+tau, duration+10+0.1, 1.0):
        #横方向のモーションプランニング
        fp = FrenetPath()
        fp.convergeTime = Ti

        # 五元方程式              始点　　　　　　　　終点　　　
        lat_qp = QuinticPolynomial(c_d, c_d_d, c_d_dd, di, 0.0, 0.0, Ti)

        #経路内のタイムステップ
        fp.t = [round(t, 3) for t in np.arange(0.0, Ti+0.1, timeStep)]
        #各タイムステップの横方向の位置、速度、加速度、躍度
        fp.d = [round(lat_qp.calc_point(t), 3) for t in fp.t]
        fp.d_d = [lat_qp.calc_first_derivative(t) for t in fp.t]
        fp.d_dd = [lat_qp.calc_second_derivative(t) for t in fp.t]
        fp.d_ddd = [lat_qp.calc_third_derivative(t) for t in fp.t]

        #縦方向のモーションプランニング (目標速度での走行を目指す場合)
        #目的速度を変化させる
        D_T_S = MAX_SPEED/5
        tvlist = np.arange(0, MAX_SPEED+0.01, D_T_S)

        tp = target_pos
        for tv in tvlist:
            tfp = copy.deepcopy(fp)

            #五元方程式               始点　　　　　　　　終点　　　
            lon_qp = QuinticPolynomial(c_s, c_s_d, c_s_dd, tp, tv, target_acc, Ti)

            #各タイムステップの縦方向の位置、速度、加速度、躍度
            tfp.s = [round(lon_qp.calc_point(t), 3) for t in fp.t]
            tfp.s_d = [round(lon_qp.calc_first_derivative(t), 3) for t in fp.t]
            tfp.s_dd = [lon_qp.calc_second_derivative(t) for t in fp.t]
            tfp.s_ddd = [lon_qp.calc_third_derivative(t) for t in fp.t]

            Jp = sum(np.power(tfp.d_ddd, 2))  # square of jerk
            Js = sum(np.power(tfp.s_ddd, 2))  # square of jerk

            #横方向のコスト　ジャークの自乗＋収束時間＋センターラインまでの距離自乗
            tfp.cd = K_J * Jp + K_T * Ti + K_D * tfp.d[-1] ** 2
            #縦方向のコスト　ジャークの自乗＋収束速度＋目的速度との差分自乗
            tfp.cv = K_J * Js + K_T * Ti + K_D * ((MAX_SPEED - tfp.s_d[-1]) ** 2)
            #コスト関数（横方向コストと縦方向コストの和）
            tfp.cf = K_LAT * tfp.cd + K_LON * tfp.cv

            frenet_paths.append(tfp)

    return frenet_paths


def calc_frenet_paths_stop(c_s, c_s_d, c_s_dd, c_d, c_d_d, c_d_dd, target_pos):
    frenet_paths = []
    target_speed = 0
    target_acc = 0

    for Ti in np.arange(minT, maxT, 1.0):
        #横方向のモーションプランニング
        fp = FrenetPath()
        fp.convergeTime = Ti

        # 五元方程式              始点　　　　　　　　終点　　　
        lat_qp = QuinticPolynomial(c_d, c_d_d, c_d_dd, c_d, 0.0, 0.0, Ti)

        # 経路内のタイムステップ
        fp.t = [round(t, 3) for t in np.arange(0.0, Ti+0.1, timeStep)]
        # 各タイムステップの横方向の位置、速度、加速度、躍度
        fp.d = [round(lat_qp.calc_point(t), 3) for t in fp.t]
        fp.d_d = [lat_qp.calc_first_derivative(t) for t in fp.t]
        fp.d_dd = [lat_qp.calc_second_derivative(t) for t in fp.t]
        fp.d_ddd = [lat_qp.calc_third_derivative(t) for t in fp.t]
        
        # 縦方向のモーションプランニング (目標速度での走行を目指す場合)
        tfp = copy.deepcopy(fp)

        #五元方程式               始点　　　　　　　　終点　　　
        lon_qp = QuinticPolynomial(c_s, c_s_d, c_s_dd, target_pos, target_speed, target_acc, Ti)

        #各タイムステップの縦方向の位置、速度、加速度、躍度
        tfp.s = [round(lon_qp.calc_point(t), 3) for t in fp.t]
        tfp.s_d = [round(lon_qp.calc_first_derivative(t), 3) for t in fp.t]
        tfp.s_dd = [round(lon_qp.calc_second_derivative(t), 3) for t in fp.t]
        tfp.s_ddd = [lon_qp.calc_third_derivative(t) for t in fp.t]

        Jp = sum(np.power(tfp.d_ddd, 2))  # square of jerk
        Js = sum(np.power(tfp.s_ddd, 2))  # square of jerk

        #横方向のコスト　ジャークの自乗＋収束時間＋センターラインまでの距離自乗
        tfp.cd = K_J * Jp + K_T * Ti + K_D * tfp.d[-1] ** 2
        #縦方向のコスト　ジャークの自乗＋収束速度＋目的地点との差分自乗
        tfp.cv = K_J * Js + K_T * Ti + K_D * ((target_pos - tfp.s[-1]) ** 2)
        #コスト関数（横方向コストと縦方向コストの和）
        tfp.cf = K_LAT * tfp.cd + K_LON * tfp.cv

        frenet_paths.append(tfp)

    return frenet_paths


def calc_frenet_paths_desired(c_s, c_s_d, c_s_dd, c_d, c_d_d, c_d_dd, route, duration=maxT):
    frenet_paths = []
    target_pos = 141.8

    # c_s_d is larger than restricted speed in intersection
    if c_s_d >= MAX_SPEED_INTERSECTION:
        for Ti in np.arange(1.0, duration+0.1, 1.0):
            #横方向のモーションプランニング
            fp = FrenetPath()
            fp.convergeTime = Ti

            #五元方程式               始点　　　　　　　　終点　　　
            lat_qp = QuinticPolynomial(c_d, c_d_d, c_d_dd, c_d, 0.0, 0.0, Ti)

            #経路内のタイムステップ
            fp.t = [round(t, 3) for t in np.arange(0.0, Ti+0.1, timeStep)]
            #各タイムステップの横方向の位置、速度、加速度、躍度
            fp.d = [round(lat_qp.calc_point(t), 3) for t in fp.t]
            fp.d_d = [lat_qp.calc_first_derivative(t) for t in fp.t]
            fp.d_dd = [lat_qp.calc_second_derivative(t) for t in fp.t]
            fp.d_ddd = [lat_qp.calc_third_derivative(t) for t in fp.t]

            # 縦方向のモーションプランニング (目標速度での走行を目指す場合)
            tfp = copy.deepcopy(fp)

            #五元方程式               始点　　　　　　　　終点　　　
            lon_qp = QuinticPolynomial(c_s, c_s_d, c_s_dd, target_pos, MAX_SPEED_INTERSECTION, 0, Ti)

            #各タイムステップの縦方向の位置、速度、加速度、躍度
            tfp.s = [round(lon_qp.calc_point(t), 3) for t in fp.t]
            tfp.s_d = [round(lon_qp.calc_first_derivative(t), 3) for t in fp.t]
            tfp.s_dd = [round(lon_qp.calc_second_derivative(t), 3) for t in fp.t]
            tfp.s_ddd = [lon_qp.calc_third_derivative(t) for t in fp.t]

            Jp = sum(np.power(tfp.d_ddd, 2))  # square of jerk
            Js = sum(np.power(tfp.s_ddd, 2))  # square of jerk

            #横方向のコスト　ジャークの自乗＋収束時間＋センターラインまでの距離自乗
            tfp.cd = K_J * Jp + K_T * Ti + K_D * tfp.d[-1] ** 2
            #縦方向のコスト　ジャークの自乗＋収束速度＋目標スピードとの差分自乗
            tfp.cv = K_J * Js + K_T * Ti + K_D * ((MAX_SPEED_INTERSECTION - tfp.s_d[-1]) ** 2)
            #コスト関数（横方向コストと縦方向コストの和）
            tfp.cf = K_LAT * tfp.cd + K_LON * tfp.cv

            frenet_paths.append(tfp)

    # c_s_d is less than restricted speed in intersection
    else:
        for Ti in np.arange(1.0, maxT+0.1, 1.0):
            #横方向のモーションプランニング
            #目的時間を変化させる
            fp = FrenetPath()

            #五元方程式               始点　　　　　　　　終点　　　
            lat_qp = QuinticPolynomial(c_d, c_d_d, c_d_dd, c_d, 0.0, 0.0, Ti)

            #経路内のタイムステップ
            fp.t = [round(t, 2) for t in np.arange(0.0, Ti+0.1, timeStep)]
            #各タイムステップの横方向の位置、速度、加速度、躍度
            fp.d = [round(lat_qp.calc_point(t), 3) for t in fp.t]
            fp.d_d = [lat_qp.calc_first_derivative(t) for t in fp.t]
            fp.d_dd = [lat_qp.calc_second_derivative(t) for t in fp.t]
            fp.d_ddd = [lat_qp.calc_third_derivative(t) for t in fp.t]

            #縦方向のモーションプランニング (目標速度での走行を目指す場合)
            tfp = copy.deepcopy(fp)
            #四元方程式               始点             終点
            lon_qp = QuarticPolynomial(c_s, c_s_d, c_s_dd, MAX_SPEED_INTERSECTION, 0.0, Ti)

            #各タイムステップの縦方向の位置、速度、加速度、躍度
            tfp.s = [round(lon_qp.calc_point(t), 3) for t in fp.t]
            tfp.s_d = [round(lon_qp.calc_first_derivative(t), 3) for t in fp.t]
            tfp.s_dd = [lon_qp.calc_second_derivative(t) for t in fp.t]
            tfp.s_ddd = [lon_qp.calc_third_derivative(t) for t in fp.t]

            Jp = sum(np.power(tfp.d_ddd, 2))  # square of jerk
            Js = sum(np.power(tfp.s_ddd, 2))  # square of jerk

            #横方向のコスト　ジャークの自乗＋収束時間＋センターラインまでの距離自乗
            tfp.cd = K_J * Jp + K_T * Ti + K_D * tfp.d[-1] ** 2
            #縦方向のコスト　ジャークの自乗＋収束速度＋目標スピードとの差分自乗
            tfp.cv = K_J * Js + K_T * Ti + K_D * ((MAX_SPEED_INTERSECTION - tfp.s_d[-1]) ** 2)
            #コスト関数（横方向コストと縦方向コストの和）
            tfp.cf = K_LAT * tfp.cd + K_LON * tfp.cv

            frenet_paths.append(tfp)     

    for fp in frenet_paths:
        if route == "BL":
            R_rem = 157.186 - fp.s[-1]
        elif route == "BR":
            R_rem = 152.167 - fp.s[-1]

        t_junc = math.ceil((R_rem/MAX_SPEED_INTERSECTION)/timeStep)
        if t_junc <= 0:
            continue

        fp.t.extend([round(fp.t[-1]+((t+1)*timeStep), 2) for t in range(t_junc)])
        fp.d.extend([fp.d[-1] for t in range(t_junc)])
        fp.d_d.extend([0.0 for t in range(t_junc)])
        fp.d_dd.extend([0.0 for t in range(t_junc)])
        fp.d_ddd.extend([0.0 for t in range(t_junc)])
        fp.s.extend([fp.s[-1]+MAX_SPEED_INTERSECTION*(t+1)*timeStep for t in range(t_junc)])
        fp.s_d.extend([MAX_SPEED_INTERSECTION for t in range(t_junc)])
        fp.s_dd.extend([0.0 for t in range(t_junc)])
        fp.s_ddd.extend([0.0 for t in range(t_junc)])

    # straight road from bottom to top
    wx = [151.6 for n in range(21)]
    wy = [-150+7.0*n for n in range(21)]
    if route == "BL":
        R = 9.8 # カーブ半径
        wx.extend([round(141.8+R*np.cos(np.radians(theta)), 2) for theta in np.arange(0, 91, 5)])
        wy.extend([round(-8.2+ R*np.sin(np.radians(theta)), 2) for theta in np.arange(0, 91, 5)])
        wx.extend([140 - i*2 for i in range(100)])
        wy.extend([1.6 for i in range(100)])
    elif route == "BR":
        R = 6.6 # カーブ半径
        wx.extend([round(158.2+R*np.cos(np.radians(theta)), 2) for theta in np.arange(180, 89, -5)])
        wy.extend([round(-8.2+ R*np.sin(np.radians(theta)), 2) for theta in np.arange(180, 89, -5)])
        wx.extend([160.0 + i*2 for i in range(100)])
        wy.extend([-1.6 for i in range(100)])        

    _, _, _, _, csp = generate_target_course(wx, wy)
    frenet_paths = calc_global_paths(frenet_paths, csp)
    frenet_paths = check_paths(frenet_paths, ob)
    if len(frenet_paths) == 0:
        return frenet_paths
    
    min_cost = float("inf")
    best_path1 = None

    for fp in frenet_paths:
        if min_cost >= fp.cf:
            min_cost = fp.cf
            best_path1 = fp


    frenet_paths = []
    for Ti in np.arange(1.0, maxT+0.1, 1.0):
        #横方向のモーションプランニング
        #目的時間を変化させる
        fp = FrenetPath()

        #五元方程式               始点　　　　　　　　終点　　　
        lat_qp = QuinticPolynomial(best_path1.d[-1], best_path1.d_d[-1], best_path1.d_dd[-1], best_path1.d[-1], 0.0, 0.0, Ti)

        #経路内のタイムステップ
        fp.t = [round(t, 2) for t in np.arange(0.0, Ti+0.1, timeStep)]
        #各タイムステップの横方向の位置、速度、加速度、躍度
        fp.d = [round(lat_qp.calc_point(t), 3) for t in fp.t]
        fp.d_d = [lat_qp.calc_first_derivative(t) for t in fp.t]
        fp.d_dd = [lat_qp.calc_second_derivative(t) for t in fp.t]
        fp.d_ddd = [lat_qp.calc_third_derivative(t) for t in fp.t]

        #縦方向のモーションプランニング (目標速度での走行を目指す場合)
        tfp = copy.deepcopy(fp)
        #四元方程式               始点             終点
        lon_qp = QuarticPolynomial(best_path1.s[-1], best_path1.s_d[-1], best_path1.s_dd[-1], MAX_SPEED, 0.0, Ti)

        #各タイムステップの縦方向の位置、速度、加速度、躍度
        tfp.s = [round(lon_qp.calc_point(t), 3) for t in fp.t]
        tfp.s_d = [round(lon_qp.calc_first_derivative(t), 3) for t in fp.t]
        tfp.s_dd = [lon_qp.calc_second_derivative(t) for t in fp.t]
        tfp.s_ddd = [lon_qp.calc_third_derivative(t) for t in fp.t]

        Jp = sum(np.power(tfp.d_ddd, 2))  # square of jerk
        Js = sum(np.power(tfp.s_ddd, 2))  # square of jerk

        #横方向のコスト　ジャークの自乗＋収束時間＋センターラインまでの距離自乗
        tfp.cd = K_J * Jp + K_T * Ti + K_D * tfp.d[-1] ** 2
        #縦方向のコスト　ジャークの自乗＋収束速度＋目標スピードとの差分自乗
        tfp.cv = K_J * Js + K_T * Ti + K_D * ((MAX_SPEED - tfp.s_d[-1]) ** 2)
        #コスト関数（横方向コストと縦方向コストの和）
        tfp.cf = K_LAT * tfp.cd + K_LON * tfp.cv

        frenet_paths.append(tfp)     

    frenet_paths = calc_global_paths(frenet_paths, csp)
    frenet_paths = check_paths(frenet_paths, ob)

    min_cost = float("inf")
    best_path2 = None

    for fp in frenet_paths:
        if min_cost >= fp.cf:
            min_cost = fp.cf
            best_path2 = fp

    best_path1.t.extend([round(best_path1.t[-1]+t, 2) for t in best_path2.t[1:]])

    best_path1.d.extend(best_path2.d[1:])
    best_path1.d_d.extend(best_path2.d_d[1:])
    best_path1.d_dd.extend(best_path2.d_dd[1:])
    best_path1.d_ddd.extend(best_path2.d_ddd[1:])

    best_path1.s.extend(best_path2.s[1:])
    best_path1.s_d.extend(best_path2.s_d[1:])
    best_path1.s_dd.extend(best_path2.s_dd[1:])
    best_path1.s_ddd.extend(best_path2.s_ddd[1:])

    best_path1.x.extend(best_path2.x[1:])
    best_path1.y.extend(best_path2.y[1:])
    best_path1.convergeTime = best_path1.t[-1]

    best_path1.isTurn = True

    return [best_path1]
    

def emergency_stop(pos_x, speed, pos_y, route):

    print("Emergency Stop")
    frenet_paths = []
    #横方向のモーションプランニング
    fp = FrenetPath()
    fp.convergeTime = 10.0
    Ti = 10.0
    #経路内のタイムステップ
    fp.t = [round(t, 3) for t in np.arange(0.0, Ti+0.1, timeStep)]
    #各タイムステップの横方向の位置、速度、加速度、躍度
    fp.d_d = [0 for t in fp.t]
    fp.d_dd = [0 for t in fp.t]
    fp.d_ddd = [0 for t in fp.t]

    #縦方向のモーションプランニング (目標速度での走行を目指す場合)
    tfp = copy.deepcopy(fp)

    count = math.ceil(speed/abs(MAX_DECEL)/timeStep)
    #各タイムステップの縦方向の位置、速度、加速度、躍度
    if route == "LR":
        xmax = pos_x + speed*(count*timeStep) + (1/2)*MAX_DECEL*((count*timeStep)**2)
        tfp.x = [pos_x+speed*t*timeStep + (1/2)*MAX_DECEL*((t*timeStep)**2) for t in range(count)]
        tfp.x.extend([xmax for i in range(len(fp.t)-count)])
        tfp.y = [pos_y for t in tfp.t]
    elif route == "RL":
        xmax = pos_x - (speed*(count*timeStep) + (1/2)*MAX_DECEL*((count*timeStep)**2))
        tfp.x = [pos_x-(speed*t*timeStep+(1/2)*MAX_DECEL*((t*timeStep)**2) ) for t in range(count)]
        tfp.x.extend([xmax for i in range(len(fp.t)-count)])
        tfp.y = [pos_y for t in tfp.t]
    elif route in ["BL", "BR"]:
        ymax = pos_y + (speed*(count*timeStep) + (1/2)*MAX_DECEL*((count*timeStep)**2))
        tfp.y = [pos_y+speed*t*timeStep+(1/2)*MAX_DECEL*((t*timeStep)**2) for t in range(count)]
        tfp.y.extend([ymax for i in range(len(fp.t)-count)])
        tfp.x = [pos_x for t in tfp.t]

    tfp.s_d = [max(speed+MAX_DECEL*t, 0) for t in fp.t]
    tfp.s_dd = [MAX_DECEL for t in range(count)]
    tfp.s_dd.extend([0.0 for i in range(len(fp.t)-count)])
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

        if any([v > max_speed+0.1 for v in fplist[i].s_d]):  #最大速度を超えないかチェック
            continue
        if any([v < -0.1 for v in fplist[i].s_d]):  #速度がマイナスにならないかチェック
            continue
        if any([a > MAX_ACCEL for a in fplist[i].s_dd]):  #最大加速度を超えないかチェック
            continue
        if any([a < MAX_DECEL for a in fplist[i].s_dd]):  #最大減速度を超えないかチェック
            continue
        
        if route in ["BR", "BL"] and fplist[i].s[-1] > targetPos:
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
            extendStep = round(maxT/timeStep - len(path.t)+1)              
            if extendStep > 0:
                # 収束以降は等速で移動すると仮定
                lastSpeed = path.s_d[-1]
                lastPosS = path.s[-1]
                path.t.extend([round(path.t[-1]+timeStep*(i+1), 3) for i in range(extendStep)])
                path.d.extend([path.d[-1] for n in range(extendStep)]) 
                path.d_d.extend([0.0 for n in range(extendStep)]) 
                path.d_dd.extend([0.0 for n in range(extendStep)]) 
                path.d_ddd.extend([0.0 for n in range(extendStep)]) 
                path.s.extend([lastPosS + lastSpeed*(n+1)*timeStep for n in range(extendStep)]) 
                path.s_d.extend([lastSpeed for n in range(extendStep)]) 
                path.s_dd.extend([0.0 for n in range(extendStep)]) 
                path.s_ddd.extend([0.0 for n in range(extendStep)])

    return fplist


def plot_path(fplist, ob=None):

    plt.figure(figsize=(8,4))
    for path in fplist:
        plt.title("position(s, t)")
        plt.plot(path.t[0:], path.s[0:])
        plt.xlabel("t")
        plt.ylabel("s")
        # plt.plot(ob[:, 0], ob[:, 1], "xk", markersize=10)

    plt.figure(figsize=(8,4))
    for path in fplist:
        plt.title("position(x, t)")
        plt.plot(path.t[0:], path.x[0:])
        plt.xlabel("t")
        plt.ylabel("x")

    plt.figure(figsize=(8,4))
    for path in fplist:
        plt.title("position(x, y)")
        plt.plot(path.x[0:], path.y[0:])
        plt.xlabel("x")
        plt.ylabel("y")
        # plt.xlim(201.6, 203.2)
        # plt.ylim(-3.2, -1.6)
        # plt.plot(ob[:, 0], ob[:, 1], "xk", markersize=10)

    plt.figure(figsize=(8,4))
    for path in fplist:
        plt.title("longitudinal velocity")
        plt.plot(path.t[0:], path.s_d[0:])
        plt.xlabel("time")
        plt.ylabel("speed")
        # plt.hlines([MAX_SPEED], 0, 10, "red", linestyles='dashed')

    plt.figure(figsize=(8,4))
    for path in fplist:    
        plt.title("longitudinal acceleration")
        plt.plot(path.t[0:], path.s_dd[0:])
        plt.xlabel("time")
        plt.ylabel("accel")
        # plt.hlines([MAX_ACCEL], 0, 10, "red", linestyles='dashed')

    # plt.figure(figsize=(8,4))
    # for path in fplist:
    #     plt.title("lateral velocity")
    #     plt.plot(path.t[0:], path.d_d[0:])

    # plt.figure(figsize=(8,4))
    # for path in fplist:    
    #     plt.title("lateral acceleration")
    #     plt.plot(path.t[0:], path.d_dd[0:])

    plt.show()


def generate_frenet_frame(objects, c_s, c_s_d, c_s_dd, c_d, mode, route, target_speed=MAX_SPEED, c_d_d=0, c_d_dd=0,\
                                target_pos=0.0, target_acc=0.0, duration=maxT, simTime=0.0, vehInstance=None):

    time_sta = time.time()

    print("mode", mode, "route", route, "duration", duration, ":c_s", c_s, ":c_s_d", c_s_d, ":c_s_dd", c_s_dd, "target_pos", '{:.5g}'.format(target_pos), "target_spped", target_speed, "time", simTime)
    if route == "LR":
        wx = [20*n for n in range(31)]
        wy = [-1.6 for n in range(31)]
    elif route == "RL":
        wx = [300-20*n for n in range(31)]
        wy = [1.6 for n in range(31)]
    elif route in ["BR", "BL"]:
        wx = [151.6 for n in range(31)]
        wy = [-150+20*n for n in range(31)]
    
    _, _, _, _, csp = generate_target_course(wx, wy)

    if mode == "free":
        fplist = calc_frenet_paths_free(c_s, c_s_d, c_s_dd, c_d, c_d_d, c_d_dd)
        fplist = interpolatePath(fplist)
        fplist = calc_global_paths(fplist, csp)
        fplist = check_paths(fplist, ob, route=route, targetPos=target_pos)

    elif mode == "follow":
        fplist = calc_frenet_paths_following(c_s, c_s_d, c_s_dd, c_d, c_d_d, c_d_dd, target_pos, target_speed, duration)
        fplist = interpolatePath(fplist)
        fplist = calc_global_paths(fplist, csp)
        fplist = check_paths(fplist, ob)

    elif mode == "yield":
        fplist = calc_frenet_paths_yield(c_s, c_s_d, c_s_dd, c_d, c_d_d, c_d_dd, target_pos, target_speed, duration)
        fplist = interpolatePath(fplist)
        fplist = calc_global_paths(fplist, csp)
        fplist = check_paths(fplist, ob)

    elif mode == "stop":
        fplist = calc_frenet_paths_stop(c_s, c_s_d, c_s_dd, c_d, c_d_d, c_d_dd, target_pos)
        fplist = interpolatePath(fplist)
        fplist = calc_global_paths(fplist, csp)
        fplist = check_paths(fplist, ob)
        
    elif mode == "turn":
        fplist = calc_frenet_paths_desired(c_s, c_s_d, c_s_dd, c_d, c_d_d, c_d_dd, route, duration=maxT)

    # plot_path(fplist)

    # 経路中の時間をシミュレーション時間に一致させる
    for path in fplist:
        path.vehID = vehInstance.id
        path.t = [round(n+simTime, 1) for n in path.t]

    tim = time.time() - time_sta
    # with open("time_generate_trajectory.csv", mode='a') as timej:
    #         timej.write(str(tim)+","+mode+"\n")

    return fplist


# 実験用
if __name__ == "__main__":

    duration = 10.0

    c_s = 100.613
    c_s_d = 10.210000000000008
    c_s_dd = 1.2000000000000455
    c_d = 0.0
    c_d_d = 0.0
    c_d_dd = 0.0
    route = "BL"
    # fplist = calc_frenet_paths_turning(c_s, c_s_d, c_s_dd, c_d, c_d_d, c_d_dd, speed_limit=2.78)
    # plot_path(fplist)

    wx = [151.6 for n in range(21)]
    wy = [-150+7.0*n for n in range(21)]

    # print("Generate Path")
    # print(":c_s", c_s, ":c_s_d", c_s_d, ":c_s_dd", c_s_dd, ":c_d", c_d, ":c_d_d", c_d_d, ":c_d_dd", c_d_dd)
    # print("targetSpeed", target_speed, ":targetPos", target_pos, ":targetAcc", target_acc, ":duration", duration)

    # カーブ部の道路セグメント
    R = 9.8 # カーブ半径
    wx.extend([round(141.8+R*np.cos(np.radians(theta)), 2) for theta in np.arange(0, 91, 5)])
    wy.extend([round(-8.2+ R*np.sin(np.radians(theta)), 2) for theta in np.arange(0, 91, 5)])
    wx.extend([140 - i*2 for i in range(100)])
    wy.extend([1.6 for i in range(100)])

    # print(wx)
    # print(wy)
    # 交差点を通過する経路
    _, _, _, _, csp = generate_target_course(wx, wy)

    fplist = calc_frenet_paths_desired(c_s, c_s_d, c_s_dd, c_d, c_d_d, c_d_dd, route, duration=maxT)
    # 交差点の手前で停止する経路
    # fplist = interpolatePath(fplist)
    # fplist = calc_global_paths(fplist, csp)
    # fplist = check_paths(fplist, ob)
    plot_path(fplist)
