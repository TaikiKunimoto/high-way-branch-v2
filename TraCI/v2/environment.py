"""シミュレーション環境（シナリオ）の定義。

環境＝形状（何が起きるか・net が決める固定構造）。負荷・規模（どれだけ）はパラメータ（総流入 Q・必須LC比率 f）。
評価環境_パターン網羅.md §4「環境＝形状／パラメータ＝負荷・規模」。コア調停（EDF/2フェーズ/Layer2）は
環境非依存で、各車は ``(target_lane, deadline_pos)`` だけを持つ（route 名 r_exit/r_pass には依存しない）。

新しい環境（合流M・封鎖B・織込み 等）は net/rou を用意し Environment を1つ追加して ENVIRONMENTS に登録する。
現状 net が存在するのは環境①（分流D）のみ。
"""

from dataclasses import dataclass


@dataclass(frozen=True)
class Group:
    """車両グループ。必須LC車は target_lane/deadline_pos を持ち、through 車（必須LCなし）は None。"""

    name: str
    route: str  # SUMO ルート id（net を通すための経路。機構は参照しない）
    weight: float  # グループ内分の重み（同種グループ間で流入を内分するのに使う）
    target_lane: int | None = None  # 必須LC の目標レーン（None=必須LCなし）
    deadline_pos: float | None = None  # 締切位置（target_lane != None のとき必須）
    depart_edge: str | None = None  # 投入 edge（None=本線 mainlane_edge）


@dataclass(frozen=True)
class Environment:
    """1シナリオの固定構造。負荷（総流入 Q・必須LC比率 f）は実行時パラメータで与える。"""

    name: str
    sumocfg: str  # config パス（cwd=TraCI からの相対）
    mainlane_edge: str  # 本線 edge（pos・締切の基準・lane_members のキー）
    mainlane_length: float  # 本線長
    groups: tuple[Group, ...]

    def group_rates(self, total_inflow: float, mlc_ratio: float) -> list[tuple[Group, float]]:
        """総流入 Q と必須LC比率 f を、グループ別の流入量[veh/h]に展開する（グループ定義順を保持）。

        必須LC車（target_lane!=None）を全体の f、through 車を (1−f) とし、同種グループ間は weight で内分する。
        単一要素環境（through+必須LC1種）なら f がそのまま必須LC比率になる。
        """
        mlc_weight = sum(g.weight for g in self.groups if g.target_lane is not None) or 1.0
        through_weight = sum(g.weight for g in self.groups if g.target_lane is None) or 1.0
        rates: list[tuple[Group, float]] = []
        for g in self.groups:
            if g.target_lane is not None:
                rates.append((g, total_inflow * mlc_ratio * (g.weight / mlc_weight)))
            else:
                rates.append((g, total_inflow * (1.0 - mlc_ratio) * (g.weight / through_weight)))
        return rates


# --- 環境① S-D 単一分流（現状唯一 net が存在する環境）---
DIVERGE = Environment(
    name="diverge",
    sumocfg="../config/high-way.sumocfg",
    mainlane_edge="MainLane1",
    mainlane_length=2500.0,
    groups=(
        Group(name="through", route="r_pass", weight=1.0),  # 直進（必須LCなし）
        Group(name="exiting", route="r_exit", weight=1.0, target_lane=2, deadline_pos=2500.0),  # 分流（目標lane2）
    ),
)

ENVIRONMENTS: dict[str, Environment] = {DIVERGE.name: DIVERGE}
