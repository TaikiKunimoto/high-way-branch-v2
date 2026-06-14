"""必須車線変更（LC）要求の統一表現と、早め固定活性化の判定。

要求 = (向き, 目標車線, 締切位置D, 残りLC回数k, 縦位置, 待ち時間)。複数シナリオ（合流M/封鎖B/分流D）を
同じ構造で表す。各車のシナリオ固有値 ``(target_lane, deadline_pos)`` は環境が生成時に与えており、本モジュールは
route 名（r_exit/r_pass）には依存しない。

活性化は「早め固定」: 締切の固定マージン手前（``ACTIVATION_MARGIN``）を通過し、かつ目標車線に未到達なら活性。
卒論の動的開始位置（渋滞末尾・前倒し）はコア外（システムモデル §4 改訂 / docs/実装計画_EDF統一調停.md §8）。
"""

from pydantic import BaseModel, ConfigDict

from status.status import CarAction
from v2.constants import ACTIVATION_MARGIN
from v2.snapshot import Snapshot, VehObs


class LCOperation(BaseModel):
    """車が完了すべき必須LC操作1つ（目標レーン・締切＋早め固定活性化の状態）。非frozen（活性化状態が変わる）。

    車は複数の操作を ``V2CAV.operations`` リストで持ち、未達成（lane != target_lane）のうち最も deadline が
    近い操作をアクティブとして要求を出す。突発障害物の回避もこのリストへ1操作 append され、元の必須LCは保持される。
    活性化状態（activated/activation_time）を内包し、待ち時間は wait_time() で求める（散在していた情報を集約）。
    """

    target_lane: int
    deadline_pos: float  # 締切位置 D
    activated: bool = False  # 活性化窓に初めて入ったら True（早め固定活性化、一度だけ）
    activation_time: float | None = None  # 活性化時刻（待ち時間の起点）

    def wait_time(self, sim_time: float) -> float:
        """活性化からの経過（EDF鍵の第2要素。未活性なら 0）。"""
        return sim_time - self.activation_time if self.activation_time is not None else 0.0


class LCRequest(BaseModel):
    """1要求車の必須LC要求。観測値（VehObs）から ``from_obs`` / ``build_all`` で生成する。"""

    model_config = ConfigDict(frozen=True)

    veh_id: str
    direction: CarAction  # 目標が現在より上のレーンなら CHANGE_LEFT（lane index 増加方向）
    current_lane: int  # 現在レーン（次の1段LCの提供車線 = current_lane + direction step を引くのに使う）
    target_lane: int
    deadline_pos: float  # D
    remaining_k: int  # |目標レーン − 現在レーン|
    current_pos: float  # 縦位置 pos
    wait_time: float  # 活性化からの経過

    @staticmethod
    def in_activation_window(
        mainlane_edge: str,
        road: str | None,
        target_lane: int | None,
        deadline_pos: float | None,
        lane: int | None,
        lane_pos: float | None,
    ) -> bool:
        """必須LCの活性化窓内か（早め固定: 締切D − ACTIVATION_MARGIN 通過後、目標車線に未到達）。"""
        if road != mainlane_edge:
            return False
        if target_lane is None or deadline_pos is None or lane is None or lane_pos is None:
            return False
        if lane == target_lane:
            return False
        return lane_pos >= deadline_pos - ACTIVATION_MARGIN

    @classmethod
    def from_obs(cls, o: VehObs, sim_time: float, mainlane_edge: str) -> "LCRequest | None":
        """観測値から活性な必須LC要求を構成する。窓外・目標到達済み・必須LCなしなら None。"""
        if not cls.in_activation_window(mainlane_edge, o.road, o.target_lane, o.deadline_pos, o.lane, o.lane_pos):
            return None
        # in_activation_window が True の時点で target_lane/deadline_pos/lane/lane_pos は非 None
        if o.target_lane is None or o.deadline_pos is None or o.lane is None or o.lane_pos is None:
            return None
        k = abs(o.target_lane - o.lane)
        direction = CarAction.CHANGE_LEFT if o.target_lane > o.lane else CarAction.CHANGE_RIGHT
        wait_time = sim_time - o.activation_time if o.activation_time is not None else 0.0
        return cls(
            veh_id=o.veh_id,
            direction=direction,
            current_lane=o.lane,
            target_lane=o.target_lane,
            deadline_pos=o.deadline_pos,
            remaining_k=k,
            current_pos=o.lane_pos,
            wait_time=wait_time,
        )

    @classmethod
    def build_all(cls, snap: Snapshot) -> "list[LCRequest]":
        """スナップショット中の全車から、活性な必須LC要求のリストを生成する。"""
        requests: list[LCRequest] = []
        for o in snap.obs.values():
            req = cls.from_obs(o, snap.sim_time, snap.mainlane_edge)
            if req is not None:
                requests.append(req)
        return requests
