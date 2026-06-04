from enum import Enum, auto


class LaneChangeStatus(Enum):
    ALL_ALLOWED = auto()  # 全ての車線変更が許可される
    SPEED_IMPROVEMENT_ONLY = auto()  # 速度向上を目的とする車線変更のみ許可
    UNAVAILABLE = auto()  # 車線変更不可


class CarStatus(Enum):
    NORMAL = auto()  # 通常
    YIELDING = auto()  # 他の車両に譲る
    LANE_CHANGING = auto()  # 車線変更中


class CarAction(Enum):
    STAY = auto()  # 車線維持
    CHANGE_LEFT = auto()  # 左車線変更
    CHANGE_RIGHT = auto()  # 右車線変更
