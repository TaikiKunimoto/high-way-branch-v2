"""v2（EDF統一調停）の定数。

v2 は完全自己完結とし、ベースライン（v1/cav）に依存しない。物理定数（v1 と同値）もここで直接定義する。
シナリオ固有の値（本線 edge・長さ・目標レーン・route）は環境ごとに異なるため ``environment.py`` が持つ。
機構依存の評価パラメータ（R, Tc, δ, 活性化マージン）は評価で確定する暫定値（`docs/実装計画_EDF統一調停.md` §7）。
"""

# --- 物理定数（v1/cav.constants と同値。v2 は自己完結のため再定義）---
MAX_SPEED: float = 27  # [m/s] 最高速度
MAX_ACCEL: float = 10.0  # [m/s^2] 最大加速度（過大・評価で見直し候補）
MAX_DECEL: float = -5.0  # [m/s^2] 最大減速度（摩擦限界）
MIN_GAP: float = 2.8  # [m] 最小車間距離
FRICTION_COEFFICIENT: float = 0.7  # 摩擦係数（制動距離計算用）
TIME_STEP: float = 0.1  # [s] シミュレーション時間ステップ

# --- 機構依存パラメータ（評価で確定・暫定値）---
R: float = 50.0  # [m] 1回のLCに残す余地（多段LCの実効距離 dist=(D−pos)−(k−1)·R）
TC: float = TIME_STEP  # [s] 調停周期（Tc=シミュレーション刻みで開始、後でstepから分離可能）
DELAY: float = 0.0  # [s] 通信遅延 δ（理想0・段階4で増加。G_req の空走項 v×δ に使う）
ACTIVATION_MARGIN: float = 400.0  # [m] 早め固定活性化: 締切Dの何m手前から要求を活性化するか
# 枠が取れず提供車も無い要求車を、締切D手前で滑らかに減速・保持し始める実効距離 dist の閾値（F5）。
# 「Dからの距離マージン」で、分岐直前まで巡航→SUMOトポロジ（lane-drop）で急停止、を避ける挙動品質用（EDF鍵には載せない）。
# 暫定100m（評価で確定）。停止距離 v_max²/(2|MAX_DECEL|)≈73m 以上かつ各環境の D 未満で機能する（現行 D: merge194/weave196/weave2392/diverge1000 はいずれも成立）。
HOLD_MARGIN: float = 100.0  # [m]（暫定・評価で確定）
