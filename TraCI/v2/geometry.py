"""分流D のシナリオ依存ジオメトリ知識を局所化する（合流M・封鎖B 追加時の差し替え点）。

ここだけが「目標レーン=2」「締切D=分岐位置」といった分流固有の前提を持つ。EDF の優先度計算
（``priority.py``）や調停（``rsu.py``）はシナリオ非依存に保つため、route→(目標レーン, 締切D) と
残りLC回数 k の写像を本モジュールに閉じ込める。
"""

from v2.constants import EXIT_ROUTE, EXIT_TARGET_LANE, MAINLANE_LENGTH


def get_target_lane(route: str | None) -> int | None:
    """必須車線変更の目標レーン。r_exit は lane2（ExitLane接続）、r_pass は必須LCなし→None。"""
    if route == EXIT_ROUTE:
        return EXIT_TARGET_LANE
    return None


def get_deadline_pos(route: str | None) -> float | None:
    """締切位置 D（MainLane1 lane_pos 基準）。分流=分岐位置（MainLane1 終端）。"""
    if route == EXIT_ROUTE:
        return float(MAINLANE_LENGTH)
    return None


def remaining_lane_changes(current_lane: int | None, target_lane: int | None) -> int | None:
    """残りLC回数 k = |目標レーン − 現在レーン|。lane0発→k=2, lane1発→k=1, lane2発→k=0。"""
    if current_lane is None or target_lane is None:
        return None
    return abs(target_lane - current_lane)
