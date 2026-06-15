#!/usr/bin/env bash
# diverge.net.xml を生成する。
#
# merge と対称の「本格分流」: 本線2車線の手前独立区間 → 下側に減速車線が現れる分流ゾーン(3車線) →
# 減速車線が出口ランプへ分岐、本線2車線は継続。ramps.guess が off-ramp の減速車線(下側 lane0)を自動付与し、
# MainApproach(MainStart->DivergeNode) の末尾 200m を分割して自動命名する:
#   MainApproach                  … 2車線の独立 approach
#   MainApproach-AddedOffRampEdge … 3車線の減速/分流ゾーン（= 調停対象 mainlane_edge）
#   MainApproach-AddedOffRampNode … 減速車線が始まるノード
# これらを DivergeZone / DivergeStart にリネームする（mainlane_edge=DivergeZone）。
set -euo pipefail
cd "$(dirname "$0")"
netconvert \
  --node-files diverge.nod.xml \
  --edge-files diverge.edg.xml \
  --ramps.guess --ramps.ramp-length 200 \
  --output-file diverge.net.xml
sed -i 's/MainApproach-AddedOffRampEdge/DivergeZone/g; s/MainApproach-AddedOffRampNode/DivergeStart/g' diverge.net.xml
echo "diverge.net.xml generated."
