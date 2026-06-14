"""EDF 優先度：実効距離 dist と調停の鍵。

優先度 = 最早締切順（EDF）。実効距離 ``dist = (D − pos) − (k − 1)·R`` が小さい（締切に余裕がない）車を上位とする。
鍵は辞書式 ``(dist小, 待ち時間大, 縦位置〔前方=大を上位〕, ID)``。第4要素 ID が一意なので鍵に同点が生じず、
処理順に循環が出ない＝デッドロックフリー（コア機構_決定事項.md §3）。種別・失敗の酷さは鍵に入れない。
"""

from v2.constants import R
from v2.lc_request import LCRequest

# 鍵: (dist, −wait_time, −lane_pos, veh_id)。タプル昇順比較で EDF 順（上位ほど緊急）になるよう各要素を符号付けする。
Key = tuple[float, float, float, int]


class EDF:
    """EDF 優先度ポリシー：実効距離 dist と調停の鍵を計算する（状態を持たない静的ロジック）。"""

    @staticmethod
    def effective_distance(request: LCRequest) -> float:
        """実効距離 dist = (D − pos) − (k − 1)·R。多段LC車(k≥2)は残り回数分だけ締切が手前に前倒しされる。"""
        return (request.deadline_pos - request.current_pos) - (request.remaining_k - 1) * R

    @staticmethod
    def make_key(request: LCRequest) -> Key:
        """調停の鍵。昇順ソートで「dist小・待ち大・前方・ID小」が上位に来る。"""
        dist = EDF.effective_distance(request)
        # −wait_time: 待ち時間が大きいほど上位 / −current_pos: 縦位置が前方（大）ほど上位 / veh_id: 最終バックストップ
        return (dist, -request.wait_time, -request.current_pos, int(request.veh_id))

    @staticmethod
    def order_requests(requests: list[LCRequest]) -> list[tuple[Key, LCRequest]]:
        """Phase A: 全要求車の鍵を同一スナップショットで計算し、EDF（鍵昇順＝dist小から）にソートして返す。"""
        keyed = [(EDF.make_key(r), r) for r in requests]
        keyed.sort(key=lambda kr: kr[0])
        return keyed
