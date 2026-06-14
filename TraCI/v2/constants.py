"""v2（EDF統一調停）の定数。

v2 は完全自己完結とし、ベースライン（v1/cav）に依存しない。物理・ジオメトリ定数（v1 と同値）も
ここで直接定義する。機構依存の評価パラメータ（R, Θ_force, Tc, δ, 活性化マージン）は評価で確定する暫定値
（`docs/実装計画_EDF統一調停.md` §7）。
"""

# --- 物理・ジオメトリ定数（v1/cav.constants と同値。v2 は自己完結のため再定義）---
MAX_SPEED: float = 27  # [m/s] 最高速度
MAX_ACCEL: float = 10.0  # [m/s^2] 最大加速度（過大・評価で見直し候補）
MAX_DECEL: float = -5.0  # [m/s^2] 最大減速度（摩擦限界）
MIN_GAP: float = 2.8  # [m] 最小車間距離
REACTION_TIME: float = 0.75  # [s] 反応時間（追従の安全車間に使用）
FRICTION_COEFFICIENT: float = 0.7  # 摩擦係数（制動距離計算用）
MAINLANE_LENGTH: float = 2500  # [m] 本線長（Start-Branch 間）
TIME_STEP: float = 0.1  # [s] シミュレーション時間ステップ

__all__ = [
    "ACTIVATION_MARGIN",
    # 渋滞末尾検出（tail position 計測用）
    "CONGESTION_SPEED",
    "DEGRADED_WAIT_SPEED",
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
DEGRADED_WAIT_SPEED: float = 5.0  # [m/s] 劣化モードの待機速度（枠なし＆締切間際で安全減速して待つ）

# --- 渋滞末尾検出（tail position 計測用。コアの活性化では使わない＝早め固定活性化）---
CONGESTION_SPEED: float = 11.1  # [m/s] 渋滞判定速度（約40km/h）
MIN_CONGESTED_VEHICLES: int = 5  # 渋滞とみなす最低車両数

# --- 分流D のジオメトリ前提 ---
EXIT_ROUTE: str = "r_exit"
PASS_ROUTE: str = "r_pass"
MAINLANE_EDGE: str = "MainLane1"
EXIT_TARGET_LANE: int = 2  # ExitLane は MainLane1_2 からのみ接続（net.xml connection fromLane=2）
