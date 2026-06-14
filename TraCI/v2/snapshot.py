"""毎Tc の全車スナップショット S_t。

2フェーズ調停（Phase A 鍵計算 / Phase B 割当）は、step 途中の traci 再読み取りをせず本スナップショットのみを
参照することで「同一スナップショットで全要求車の鍵を計算」（コア機構_決定事項.md §5）を保証する。
``lane_members`` は目標車線の後続車を高速に引くための縦位置降順インデックス。
"""

from dataclasses import dataclass
from typing import TYPE_CHECKING

from v2.constants import MAINLANE_EDGE

if TYPE_CHECKING:
    from v2.v2_cav import V2CAV


@dataclass(frozen=True)
class VehObs:
    """1車両の観測値（S_t での凍結値）。"""

    veh_id: str
    route: str | None
    road: str | None
    lane: int | None
    lane_pos: float | None
    speed: float
    activation_time: float | None


@dataclass(frozen=True)
class Snapshot:
    sim_time: float
    obs: dict[str, VehObs]
    lane_members: dict[str, list[str]]  # lane_id -> 縦位置降順の車両idリスト


def _lane_pos_or_zero(o: VehObs) -> float:
    return o.lane_pos if o.lane_pos is not None else 0.0


def capture(vehicles: "list[V2CAV]", sim_time: float) -> Snapshot:
    """走行中の全車両の観測値を凍結し、車線別の縦位置降順インデックスを作る。"""
    obs: dict[str, VehObs] = {}
    lane_members: dict[str, list[str]] = {}
    for veh in vehicles:
        p = veh.params
        obs[p.id] = VehObs(p.id, p.route, p.road, p.lane, p.lane_pos, p.speed, p.activation_time)
        if p.road == MAINLANE_EDGE and p.lane is not None:
            lane_members.setdefault(f"{p.road}_{p.lane}", []).append(p.id)
    for ids in lane_members.values():
        ids.sort(key=lambda i: _lane_pos_or_zero(obs[i]), reverse=True)
    return Snapshot(sim_time, obs, lane_members)
