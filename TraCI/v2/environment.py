"""シミュレーション環境（シナリオ）の定義。

環境＝形状（何が起きるか・net が決める固定構造）。負荷・規模（どれだけ）はパラメータ（総流入 Q・必須LC比率 f）。
評価環境_パターン網羅.md §4「環境＝形状／パラメータ＝負荷・規模」。コア調停（EDF/2フェーズ/Layer2）は
環境非依存で、各車は ``(target_lane, deadline_pos)`` だけを持つ（route 名 r_exit/r_pass には依存しない）。

新しい環境（合流M・封鎖B・織込み 等）は net/rou を用意し Environment を1つ追加して ENVIRONMENTS に登録する。
現状 net が存在するのは環境①（分流D）のみ。
"""

from typing import NamedTuple

from pydantic import BaseModel, ConfigDict


class Group(BaseModel):
    """車両グループ。必須LC車は target_lane/deadline_pos を持ち、through 車（必須LCなし）は None。"""

    model_config = ConfigDict(frozen=True)

    name: str
    route: str  # SUMO ルート id（net を通すための経路。機構は参照しない）
    weight: float  # グループ内分の重み（同種グループ間で流入を内分するのに使う）
    target_lane: int | None = None  # 必須LC の目標レーン（None=必須LCなし）
    deadline_pos: float | None = None  # 締切位置（target_lane != None のとき必須）
    depart_edge: str | None = None  # 投入 edge（None=本線 mainlane_edge）
    depart_lanes: tuple[int, ...] | None = None  # 投入レーン候補（None=投入edgeの全レーン）


class GroupRate(NamedTuple):
    """グループと、その展開後の流入量のペア（group_rates の結果）。``for group, rate in ...`` のタプル展開も可。"""

    group: Group
    rate: float  # 流入量 [veh/h]


class Environment(BaseModel):
    """1シナリオの固定構造。負荷（総流入 Q・必須LC比率 f）は実行時パラメータで与える。"""

    model_config = ConfigDict(frozen=True)

    name: str
    sumocfg: str  # config パス（cwd=TraCI からの相対）
    mainlane_edge: str  # 本線 edge（pos・締切の基準・lane_members のキー）
    mainlane_length: float  # 本線長
    groups: tuple[Group, ...]

    def group_rates(self, total_inflow: float, mlc_ratio: float) -> list[GroupRate]:
        """総流入 Q と必須LC比率 f を、グループ別の流入量[veh/h]に展開する（グループ定義順を保持）。

        必須LC車（target_lane!=None）を全体の f、through 車を (1−f) とし、同種グループ間は weight で内分する。
        単一要素環境（through+必須LC1種）なら f がそのまま必須LC比率になる。必須LCグループが無い環境
        （straight 等。障害物は --obstacle で動的付与）では f を無視し全車を through にする。
        """
        has_mlc = any(g.target_lane is not None for g in self.groups)
        f = mlc_ratio if has_mlc else 0.0
        mlc_weight = sum(g.weight for g in self.groups if g.target_lane is not None) or 1.0
        through_weight = sum(g.weight for g in self.groups if g.target_lane is None) or 1.0
        rates: list[GroupRate] = []
        for g in self.groups:
            if g.target_lane is not None:
                rates.append(GroupRate(g, total_inflow * f * (g.weight / mlc_weight)))
            else:
                rates.append(GroupRate(g, total_inflow * (1.0 - f) * (g.weight / through_weight)))
        return rates


# --- 環境① S-D 単一分流（実分流形状）。手前に本線2車線が独立する区間があり、DivergeStart から本線下側に
# 減速車線(DivergeZone lane0)が現れ、DivergeNode で出口ランプ(ExitRamp)へ分岐。本線2車線は継続。
# 分流車は 本線→減速車線 lane0 へ必須LC（下向き）し出口へ。net は ramps.guess 生成（config/v2/diverge/build.sh）---
DIVERGE = Environment(
    name="diverge",
    sumocfg="../config/v2/diverge/diverge.sumocfg",
    mainlane_edge="DivergeZone",  # 減速車線を含む3車線の分流ゾーン（調停対象）
    mainlane_length=194.0,  # 減速車線(DivergeZone lane0)が出口へ分岐する位置 ＝ 締切
    groups=(
        # 直進（必須LCなし）。本線2車線（MainApproach → DivergeZone lane1/2 → MainLane）
        Group(name="through", route="r_pass", weight=1.0, depart_edge="MainApproach"),
        # 分流（本線 → 減速車線 DivergeZone lane0 へ必須LC、締切=分岐位置）→ 出口ランプへ
        Group(name="exiting", route="r_exit", weight=1.0, target_lane=0, deadline_pos=194.0, depart_edge="MainApproach"),
    ),
)

# --- 環境② S-M 単一合流（始端の加速車線=MergeZone_0 が drop。加速車線の車は Lane1 へ必須合流）---
MERGE = Environment(
    name="merge",
    sumocfg="../config/v2/merge/merge.sumocfg",
    mainlane_edge="MergeZone",
    mainlane_length=200.0,  # 加速車線が消える位置 ＝ 締切
    groups=(
        # 直進（必須LCなし）。本線 lane 1/2/3（lane0 は加速車線）
        Group(name="through", route="r_main", weight=1.0, depart_lanes=(1, 2, 3)),
        # 合流（加速車線 lane0 → 目標 lane1 へ必須LC、締切=加速車線端 200m）
        Group(name="merging", route="r_main", weight=1.0, target_lane=1, deadline_pos=200.0, depart_lanes=(0,)),
    ),
)

# --- 環境③ 素地 = 直進3車線（straight）。障害物Bは --obstacle で動的に発生させる（突発タイミング＝パラメータ）---
# 例: env③ S-B1 = `--env straight --obstacle 1,1500,80`（Lane1・pos1500・t80 で停止車両を発生）
STRAIGHT = Environment(
    name="straight",
    sumocfg="../config/v2/straight/straight.sumocfg",
    mainlane_edge="Road",
    mainlane_length=2500.0,
    groups=(Group(name="through", route="r_main", weight=1.0),),  # 全車 through（必須LCは障害物で動的付与）
)

# --- 環境⑤ MD-2 両側織込み（4車線 WeaveZone: lane0=加速車線(下)・lane1-3=本線、top本線 lane3→出口(上)）---
WEAVE2 = Environment(
    name="weave2",
    sumocfg="../config/v2/weave2/weave2.sumocfg",
    mainlane_edge="WeaveZone",
    mainlane_length=2300.0,  # 加速車線drop＝出口分岐位置＝締切
    groups=(
        # 直進（必須LCなし）。本線 lane1-3
        Group(name="through", route="r_main", weight=1.0, depart_lanes=(1, 2, 3)),
        # 合流（加速車線 lane0 → 本線 lane1 へ）
        Group(name="merging", route="r_main", weight=1.0, target_lane=1, deadline_pos=2300.0, depart_lanes=(0,)),
        # 分流（本線 lane1/2 → top lane3 → 出口へ）。merge と逆向きに横断＝織込み
        Group(name="diverging", route="r_exit", weight=1.0, target_lane=3, deadline_pos=2300.0, depart_lanes=(1, 2)),
    ),
)

# --- 環境④ MD-1f 織込み（補助車線、一側）。WeaveZone 3車線: lane0=補助車線・lane1,2=本線2車線。---
# 補助車線(lane0)は端で出口(decel)へ。合流車は lane0→lane1 へ抜け、分流車は lane1,2→lane0 へ降りる＝逆向き織込み。
WEAVE = Environment(
    name="weave",
    sumocfg="../config/v2/weave/weave.sumocfg",
    mainlane_edge="WeaveZone",
    mainlane_length=2000.0,
    groups=(
        # 直進（必須LCなし）。本線 lane1,2
        Group(name="through", route="r_main", weight=1.0, depart_lanes=(1, 2)),
        # 合流（補助車線 lane0 → 本線 lane1 へ抜ける。lane0 は出口へ繋がるため抜けないと本線へ行けない）
        Group(name="merging", route="r_main", weight=1.0, target_lane=1, deadline_pos=2000.0, depart_lanes=(0,)),
        # 分流（本線 lane1,2 → 補助車線 lane0 へ降りて出口）。merge と逆向き＝織込み
        Group(name="diverging", route="r_exit", weight=1.0, target_lane=0, deadline_pos=2000.0, depart_lanes=(1, 2)),
    ),
)

ENVIRONMENTS: dict[str, Environment] = {e.name: e for e in (DIVERGE, MERGE, STRAIGHT, WEAVE2, WEAVE)}
