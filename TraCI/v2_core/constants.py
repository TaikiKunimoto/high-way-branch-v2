"""v2（EDF統一調停）の定数。

機構非依存の物理・ジオメトリ定数は ``cav.constants`` から re-import し single source of truth を保つ。
機構依存の評価パラメータ（R, Θ_force, Tc, δ, 活性化マージン）はここで新規定義する
（数値は評価で確定する暫定値。`docs/実装計画_EDF統一調停.md` §7）。
"""

from cav.constants import (
    FRICTION_COEFFICIENT,
    MAINLANE_LENGTH,
    MAX_ACCEL,
    MAX_DECEL,
    MAX_SPEED,
    MIN_GAP,
    REACTION_TIME,
    TIME_STEP,
)

__all__ = [
    "ACTIVATION_MARGIN",
    # 渋滞末尾検出（tail position 計測用）
    "CONGESTION_SPEED",
    "DELAY",
    # 分流D のジオメトリ前提
    "EXIT_ROUTE",
    "EXIT_TARGET_LANE",
    # cav.constants から re-export する機構非依存定数
    "FRICTION_COEFFICIENT",
    "MAINLANE_EDGE",
    "MAINLANE_LENGTH",
    "MAX_ACCEL",
    "MAX_DECEL",
    "MAX_SPEED",
    "MIN_CONGESTED_VEHICLES",
    "MIN_GAP",
    "PASS_ROUTE",
    "REACTION_TIME",
    "TC",
    "THETA_FORCE",
    "TIME_STEP",
    # 機構依存パラメータ
    "R",
]

# --- 機構依存パラメータ（評価で確定・暫定値）---
R: float = 50.0  # [m] 1回のLCに残す余地（多段LCの実効距離 dist=(D−pos)−(k−1)·R）
THETA_FORCE: float = 50.0  # [m] dist≤Θ_force かつ枠なし→劣化モード（鍵には入れない物理閾値）
TC: float = TIME_STEP  # [s] 調停周期（Tc=シミュレーション刻みで開始、後でstepから分離可能）
DELAY: float = 0.0  # [s] 通信遅延 δ（理想0・段階4で増加。G_req の空走項 v×δ に使う）
ACTIVATION_MARGIN: float = 400.0  # [m] 早め固定活性化: 締切Dの何m手前から要求を活性化するか

# --- 渋滞末尾検出（tail position 計測用。コアの活性化では使わない＝早め固定活性化）---
CONGESTION_SPEED: float = 11.1  # [m/s] 渋滞判定速度（約40km/h）
MIN_CONGESTED_VEHICLES: int = 5  # 渋滞とみなす最低車両数

# --- 分流D のジオメトリ前提 ---
EXIT_ROUTE: str = "r_exit"
PASS_ROUTE: str = "r_pass"
MAINLANE_EDGE: str = "MainLane1"
EXIT_TARGET_LANE: int = 2  # ExitLane は MainLane1_2 からのみ接続（net.xml connection fromLane=2）
