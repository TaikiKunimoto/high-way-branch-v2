"""Layer1（RSU）の Phase B: 鍵順の提供車割当。

鍵（Phase A の EDF 順）だけではデッドロックフリーは出ない。鍵 ＋「順序通りの割当」で初めて出る
（コア機構_決定事項.md §5）。鍵順（dist 昇順＝最も緊急から）に処理し、各要求車に対し
「次の1段LCの目標車線の後続のうち、鍵が自分より下位（劣位）かつ今Tc未占有 の最近傍」を提供車に確保する。

- 占有印（claimed）で同一提供車の二重割当を防ぐ＝**横取り禁止**。
- 既に上位車の提供車として確保された要求車は、今Tc は譲る側に回り自分のLCは見送る（譲歩の伝播・§6）。
- 要求車が停車中（speed=0）は2番目に近い候補を選ぶ（先頭直後に詰まるのを避ける既存例外）。
"""

from dataclasses import dataclass

from status.status import CarAction
from v2_core.constants import MAINLANE_EDGE
from v2_core.lc_request import LCRequest
from v2_core.priority import Key
from v2_core.snapshot import Snapshot


@dataclass(frozen=True)
class Assignment:
    requester_id: str
    provider_id: str


def arbitrate(keyed: list[tuple[Key, LCRequest]], snap: Snapshot) -> list[Assignment]:
    """Phase B。鍵昇順（dist小から）に提供車を占有印つきで確保し、割当のリストを返す。"""
    request_key: dict[str, Key] = {req.veh_id: key for key, req in keyed}
    claimed: set[str] = set()
    assignments: list[Assignment] = []
    for key, req in keyed:
        if req.veh_id in claimed:
            # 既に上位車の提供車として確保済み → 今Tc は譲る側。自分のLCは見送る（譲歩の伝播）
            continue
        provider = _find_provider(req, key, snap, claimed, request_key)
        if provider is not None:
            claimed.add(provider)  # 占有印（横取り禁止）
            assignments.append(Assignment(req.veh_id, provider))
        # provider が無い＝譲れる枠なし → B6 で Θ_force 劣化につなぐ
    return assignments


def _find_provider(
    req: LCRequest, my_key: Key, snap: Snapshot, claimed: set[str], request_key: dict[str, Key]
) -> str | None:
    """次の1段LCの目標車線の後続から、鍵劣位かつ未占有の最近傍（停車中は2番目）を選ぶ。"""
    step = 1 if req.direction == CarAction.CHANGE_LEFT else -1
    next_lane = req.current_lane + step
    members = snap.lane_members.get(f"{MAINLANE_EDGE}_{next_lane}", [])  # 縦位置降順

    viable: list[str] = []
    for vid in members:
        o = snap.obs[vid]
        if o.lane_pos is None or o.lane_pos >= req.current_pos:
            continue  # 後続（自分より後ろ）のみ
        if vid in claimed:
            continue  # 占有印（横取り禁止）
        other_key = request_key.get(vid)
        if other_key is not None and other_key < my_key:
            continue  # 相手が自分より緊急（鍵上位）→ 譲ってもらえない。要求なし車は常に譲れる
        viable.append(vid)

    if not viable:
        return None
    # members は縦位置降順なので viable も降順（先頭＝最近傍）
    if snap.obs[req.veh_id].speed != 0:
        return viable[0]
    return viable[1] if len(viable) > 1 else None
