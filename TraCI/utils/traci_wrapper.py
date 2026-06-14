"""TraCI 関数の薄いラッパ。

戻り値を ``cast`` で型付けし、mypy strict 下でも扱いやすくする。v1/v2 共有。
位置・距離は [m]、速度は [m/s]、時刻は [s]。
"""

from typing import cast

import traci


# Vehicle系の関数
def get_veh_id_list() -> list[str]:
    """シミュレーション中（ネットワーク上）の全車両IDのリストを返す。"""
    return cast(list[str], traci.vehicle.getIDList())


def get_veh_departure(id: str) -> float:
    """車両の出発（投入）時刻 [s] を返す。"""
    return cast(float, traci.vehicle.getDeparture(id))


def get_veh_type(id: str) -> str:
    """車両のタイプID（例 "CAV"）を返す。"""
    return cast(str, traci.vehicle.getTypeID(id))


def get_veh_pos(id: str) -> tuple[float, float]:
    """車両の (x, y) 座標 [m] を返す。"""
    return cast(tuple[float, float], traci.vehicle.getPosition(id))


def get_veh_lane_position(id: str) -> float:
    """車両のレーン内縦位置 [m]（現在 edge の始端からの距離）を返す。"""
    return cast(float, traci.vehicle.getLanePosition(id))


def get_veh_route_id(id: str) -> str:
    """車両のルートID（例 "r_pass" / "r_exit"）を返す。"""
    return cast(str, traci.vehicle.getRouteID(id))


def get_veh_lane_id(id: str) -> str:
    """車両の現在レーンID（例 "MainLane1_0"）を返す。"""
    return cast(str, traci.vehicle.getLaneID(id))


def get_veh_lane_index(id: str) -> int:
    """車両の現在レーンインデックス（0 始まり。0=最下段）を返す。"""
    return cast(int, traci.vehicle.getLaneIndex(id))


def get_veh_speed(id: str) -> float:
    """車両の現在速度 [m/s] を返す。"""
    return cast(float, traci.vehicle.getSpeed(id))


def get_veh_acceleration(id: str) -> float:
    """車両の現在加速度 [m/s^2] を返す。"""
    return cast(float, traci.vehicle.getAcceleration(id))


def get_veh_leader(id: str, setting: int) -> tuple[str, float] | None:
    """前方車両の (ID, 車間距離 [m]) を返す。``setting`` は探索の look-ahead 距離 [m]。前方車がいなければ None。"""
    return cast(tuple[str, float] | None, traci.vehicle.getLeader(id, setting))


def get_veh_road_id(id: str) -> str:
    """車両の現在 edge ID（例 "MainLane1"）を返す。"""
    return cast(str, traci.vehicle.getRoadID(id))


# Simulation系の関数
def get_sim_departed_veh_id_list() -> list[str]:
    """直近 step で投入（出発）された車両IDのリストを返す。"""
    return cast(list[str], traci.simulation.getDepartedIDList())


def get_sim_arrived_veh_id_list() -> list[str]:
    """直近 step で範囲外に出た（到着した）車両IDのリストを返す。"""
    return cast(list[str], traci.simulation.getArrivedIDList())


def get_sim_time() -> float:
    """現在のシミュレーション時刻 [s] を返す。"""
    return cast(float, traci.simulation.getTime())


# Lane系の関数
def get_lane_max_speed(lane_id: str) -> float:
    """レーンの制限速度 [m/s] を返す。"""
    return cast(float, traci.lane.getMaxSpeed(lane_id))


def get_lane_last_step_veh_ids(lane_id: str) -> list[str]:
    """直近 step でそのレーン上にいた車両IDのリストを返す。"""
    return cast(list[str], traci.lane.getLastStepVehicleIDs(lane_id))
