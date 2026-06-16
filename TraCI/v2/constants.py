"""v2（EDF統一調停）の定数。

v2 は完全自己完結とし、ベースライン（v1/cav）に依存しない。物理定数（v1 と同値）もここで直接定義する。
シナリオ固有の値（本線 edge・長さ・目標レーン・route）は環境ごとに異なるため ``environment.py`` が持つ。
機構依存の評価パラメータ（R, Tc, δ, 活性化マージン）は評価で確定する暫定値（`docs/実装計画_EDF統一調停.md` §7）。
"""

# --- 物理定数（提案/ベースラインで統一。vType(accel/decel/minGap/length) と同値にする）---
MAX_SPEED: float = 27  # [m/s] 最高速度
# 最大加速度。提案は traci 制御（speedMode 0）で vType の accel が直接指令に効かないため、本値が実効上限になる
# （_safe_accel_duration → slowDown の継続時間で accel=MAX_ACCEL にクリップ）。2.6 は SUMO passenger 既定／
# IDM 現実レンジ（Treiber&Kesting 0.8–2.5）に基づく文献値（旧 10.0 は物理限界超の非現実値）。評価で 2.0/2.6/3.0 を感度分析。
MAX_ACCEL: float = 2.6  # [m/s^2] 最大加速度（vType accel と統一）
MAX_DECEL: float = -5.0  # [m/s^2] 最大減速度（vType decel=5.0 と統一）
MIN_GAP: float = 2.8  # [m] 最小車間距離（vType minGap と統一。提案は model_post_init で setMinGap）
FRICTION_COEFFICIENT: float = 0.7  # 摩擦係数（制動距離計算用）
TIME_STEP: float = 0.1  # [s] シミュレーション時間ステップ

# --- 機構依存パラメータ（評価で確定・暫定値）---
R: float = 50.0  # [m] 1回のLCに残す余地（多段LCの実効距離 dist=(D−pos)−(k−1)·R）
TC: float = TIME_STEP  # [s] 調停周期（Tc=シミュレーション刻みで開始、後でstepから分離可能）
DELAY: float = 0.0  # [s] 通信遅延 δ（理想0・段階4で増加。G_req の空走項 v×δ に使う）
ACTIVATION_MARGIN: float = 400.0  # [m] 早め固定活性化: 締切Dの何m手前から要求を活性化するか
