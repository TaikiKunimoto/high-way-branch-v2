"""毎Tc の全車スナップショット S_t。

2フェーズ調停（Phase A 鍵計算 / Phase B 割当）は、step 途中の traci 再読み取りをせず本スナップショットのみを
参照することで「同一スナップショットで全要求車の鍵を計算」（コア機構_決定事項.md §5）を保証する。
``lane_members`` は目標車線の後続車を高速に引くための縦位置降順インデックス。各車は環境が与える
``target_lane``/``deadline_pos``（必須LC仕様）を持ち、route 名には依存しない（環境非依存）。
``mainlane_edge`` を持たせることで、下流（rsu/safety/lc_request）は環境を意識せず本線 edge を引ける。
"""

from dataclasses import dataclass
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from v2.v2_cav import V2CAV


@dataclass(frozen=True)
class VehObs:
    """1車両の観測値（S_t での凍結値）。"""

    veh_id: str
    target_lane: int | None  # 必須LC の目標レーン（None=必須LCなし）
    deadline_pos: float | None  # 締切位置
    road: str | None
    lane: int | None
    lane_pos: float | None
    speed: float
    activation_time: float | None


@dataclass(frozen=True)
class Snapshot:
    sim_time: float
    mainlane_edge: str  # 本線 edge（lane_members のキー・活性化窓判定の基準）
    obs: dict[str, VehObs]
    lane_members: dict[str, list[str]]  # lane_id -> 縦位置降順の車両idリスト


def _lane_pos_or_zero(o: VehObs) -> float:
    return o.lane_pos if o.lane_pos is not None else 0.0


def capture(vehicles: "list[V2CAV]", sim_time: float, mainlane_edge: str) -> Snapshot:
    """走行中の全車両の観測値を凍結し、本線 edge の車線別 縦位置降順インデックスを作る。"""
    obs: dict[str, VehObs] = {}
    lane_members: dict[str, list[str]] = {}
    for veh in vehicles:
        p = veh.params
        obs[p.id] = VehObs(p.id, p.target_lane, p.deadline_pos, p.road, p.lane, p.lane_pos, p.speed, p.activation_time)
        if p.road == mainlane_edge and p.lane is not None:
            lane_members.setdefault(f"{p.road}_{p.lane}", []).append(p.id)
    for ids in lane_members.values():
        ids.sort(key=lambda i: _lane_pos_or_zero(obs[i]), reverse=True)
    return Snapshot(sim_time, mainlane_edge, obs, lane_members)
