"""simple_cav / custom_cav で値が一致する共通定数。

手法ごとに異なる定数は各CAVファイル側で個別に定義する:
- LANE_CHANGE_MARGIN_CONGESTED（simple=120.0 / custom=70.0）
- default_cav の MAX_ACCEL=3.0 / MIN_GAP=2.5 / MERGE_STAET_POS=2100
"""

MAX_SPEED = 27  # [m/s]
MAX_ACCEL = 10.0  # [m/ss]
MAX_DECEL = -5.0  # [m/ss]
MIN_GAP = 2.8  # [m]
REACTION_TIME = 0.75  # [s]
FRICTION_COEFFICIENT = 0.7  # 摩擦係数
LANE_WIDTH = 3.2  # [m]
LANE_CHANGE_MARGIN_DEFAULT = 400.0  # [m] 通常時に分岐地点の何メートル手前から車線変更を許可するか
SPEED_IMPROVEMENT_THRESHOLD = 40.0  # 車線変更による速度改善の閾値 [%]
MAINLANE_LENGTH = 2500  # [m]
TIME_STEP = 0.1  # [s]
