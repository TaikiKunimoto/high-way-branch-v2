"""挿入時の安全判定（前後二方向）と安全ギャップ G_req（式C）。

G_req = 空走(v×δ) + 制動距離 + minGap。人間の反応時間(0.75s)は除き、空走は通信遅延 δ（理想 δ=0 でゼロ）。
挿入は「次の1段LCの目標車線で、前後の必要車間を満たす」ことを条件とする（custom_cav の前後二方向チェックを移植）。
"""

from status.status import CarAction
from v2.constants import DELAY, FRICTION_COEFFICIENT, MAX_SPEED, MIN_GAP
from v2.lc_request import LCRequest
from v2.snapshot import Snapshot

VEH_LENGTH = 5.0  # 車長 [m]（vType length と一致）


class Safety:
    """安全層：安全ギャップ G_req（式C）と、挿入時の前後二方向チェック（状態を持たない静的ロジック）。"""

    @staticmethod
    def g_req(speed: float) -> float:
        """安全ギャップ G_req（式C）= 空走(v×δ) + 制動距離 + minGap。δ=0（理想通信）で空走項はゼロ。"""
        speed_kmh = speed * 3.6
        reaction = speed * DELAY
        braking = (speed_kmh**2) / (254.016 * FRICTION_COEFFICIENT)
        return reaction + braking + MIN_GAP

    @staticmethod
    def next_lane(req: LCRequest) -> int:
        """次の1段LCの目標車線（current_lane を direction 方向に1つ）。"""
        return req.current_lane + (1 if req.direction == CarAction.CHANGE_LEFT else -1)

    @staticmethod
    def is_insertion_safe(req: LCRequest, req_speed: float, snap: Snapshot) -> bool:
        """次の1段LCの目標車線へ瞬時LCしても前後の必要車間を満たすか（前後二方向・全車走査）。

        最近傍だけでなく目標車線の全車を走査する。縦位置が自分とほぼ同じ車は leader 側（gap≈0）に倒れて
        必要車間を割るため不可となり、同一縦位置への重なり挿入（側面衝突）を防ぐ。
        """
        for vid in snap.lane_members.get(f"{snap.mainlane_edge}_{Safety.next_lane(req)}", []):
            o = snap.obs[vid]
            if o.lane_pos is None:
                continue
            if o.lane_pos < req.current_pos:
                # 後続（follower）。後続が速いほど大きな車間が要る
                gap = req.current_pos - o.lane_pos
                speed_diff = o.speed - req_speed
                required = VEH_LENGTH + MIN_GAP * 1.5
                if speed_diff > 0:
                    required += Safety.g_req(o.speed) * (speed_diff / MAX_SPEED)
            else:
                # 先行（leader、同一縦位置を含む）。自車が速いほど大きな車間が要る
                gap = o.lane_pos - req.current_pos
                speed_diff = req_speed - o.speed
                required = VEH_LENGTH + MIN_GAP * 1.5
                if speed_diff > 0:
                    required += Safety.g_req(req_speed) * (speed_diff / MAX_SPEED)
            if gap < required:
                return False
        return True
