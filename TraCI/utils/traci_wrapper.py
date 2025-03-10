from typing import cast

import traci

# TraCIの関数をラップする関数を定義する


# Vehicle系の関数
def get_veh_id_list() -> list[str]:
    return cast(list[str], traci.vehicle.getIDList())


def get_veh_departure(id: str) -> float:
    return cast(float, traci.vehicle.getDeparture(id))


def get_veh_type(id: str) -> str:
    return cast(str, traci.vehicle.getTypeID(id))


def get_veh_pos(id: str) -> tuple[float, float]:
    return cast(tuple[float, float], traci.vehicle.getPosition(id))


def get_veh_lane_position(id: str) -> float:
    return cast(float, traci.vehicle.getLanePosition(id))


def get_veh_route_id(id: str) -> str:
    return cast(str, traci.vehicle.getRouteID(id))


def get_veh_lane_id(id: str) -> str:
    return cast(str, traci.vehicle.getLaneID(id))


def get_veh_lane_index(id: str) -> int:
    return cast(int, traci.vehicle.getLaneIndex(id))


def get_veh_speed(id: str) -> float:
    return cast(float, traci.vehicle.getSpeed(id))


def get_veh_acceleration(id: str) -> float:
    return cast(float, traci.vehicle.getAcceleration(id))


def get_veh_leader(id: str, setting: int) -> tuple[str, float] | None:
    return cast(tuple[str, float] | None, traci.vehicle.getLeader(id, setting))


def get_veh_road_id(id: str) -> str:
    return cast(str, traci.vehicle.getRoadID(id))


# Simulation系の関数
def get_sim_departed_veh_id_list() -> list[str]:
    return cast(list[str], traci.simulation.getDepartedIDList())


def get_sim_arrived_veh_id_list() -> list[str]:
    return cast(list[str], traci.simulation.getArrivedIDList())


def get_sim_time() -> float:
    return cast(float, traci.simulation.getTime())


# Lane系の関数
def get_lane_max_speed(lane_id: str) -> float:
    return cast(float, traci.lane.getMaxSpeed(lane_id))


def get_lane_last_step_veh_ids(lane_id: str) -> list[str]:
    return cast(list[str], traci.lane.getLastStepVehicleIDs(lane_id))
