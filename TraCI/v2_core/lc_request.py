"""必須車線変更（LC）要求の統一表現と、早め固定活性化の判定。

要求 = (向き, 目標車線, 締切位置D, 残りLC回数k, 縦位置, 待ち時間)。複数シナリオ（合流M/封鎖B/分流D）を
同じ構造で表すための統一表現で、シナリオ固有の D・目標車線は ``geometry`` が供給する。

活性化は「早め固定」: 締切の固定マージン手前（``ACTIVATION_MARGIN``）を通過し、かつ目標車線に未到達なら活性。
卒論の動的開始位置（渋滞末尾・前倒し）はコア外（システムモデル §4 改訂 / docs/実装計画_EDF統一調停.md §8）。
"""

from dataclasses import dataclass

from status.status import CarAction
from v2_core import geometry
from v2_core.constants import ACTIVATION_MARGIN, MAINLANE_EDGE
from v2_core.snapshot import Snapshot, VehObs


@dataclass(frozen=True)
class LCRequest:
    """1要求車の必須LC要求。"""

    veh_id: str
    direction: CarAction  # 分流は CHANGE_LEFT（lane index 増加方向）
    current_lane: int  # 現在レーン（次の1段LCの提供車線 = current_lane + direction step を引くのに使う）
    target_lane: int
    deadline_pos: float  # D
    remaining_k: int  # |目標レーン − 現在レーン|
    current_pos: float  # 縦位置 pos
    wait_time: float  # 活性化からの経過


def in_activation_window(road: str | None, route: str | None, lane: int | None, lane_pos: float | None) -> bool:
    """必須LCの活性化窓内か（早め固定: 締切D − ACTIVATION_MARGIN 通過後、目標車線に未到達）。"""
    if road != MAINLANE_EDGE:
        return False
    target = geometry.get_target_lane(route)
    deadline = geometry.get_deadline_pos(route)
    if target is None or deadline is None or lane is None or lane_pos is None:
        return False
    if lane == target:
        return False
    return lane_pos >= deadline - ACTIVATION_MARGIN


def active_request(o: VehObs, sim_time: float) -> LCRequest | None:
    """観測値から活性な必須LC要求を構成する。窓外・目標到達済み・対象外（r_pass等）なら None。"""
    if not in_activation_window(o.road, o.route, o.lane, o.lane_pos):
        return None
    target = geometry.get_target_lane(o.route)
    deadline = geometry.get_deadline_pos(o.route)
    # in_activation_window が True の時点で target/deadline/lane/lane_pos は非 None
    if target is None or deadline is None or o.lane is None or o.lane_pos is None:
        return None
    k = abs(target - o.lane)
    direction = CarAction.CHANGE_LEFT if target > o.lane else CarAction.CHANGE_RIGHT
    wait_time = sim_time - o.activation_time if o.activation_time is not None else 0.0
    return LCRequest(o.veh_id, direction, o.lane, target, deadline, k, o.lane_pos, wait_time)


def build_requests(snap: Snapshot) -> list[LCRequest]:
    """スナップショット中の全車から、活性な必須LC要求のリストを生成する。"""
    requests: list[LCRequest] = []
    for o in snap.obs.values():
        req = active_request(o, snap.sim_time)
        if req is not None:
            requests.append(req)
    return requests
