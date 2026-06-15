#!/usr/bin/env bash
# merge.net.xml を生成する。
#
# 他環境（weave 等）は素の netconvert で済むが、merge は「独立した合流ランプ→加速車線(下側)→drop」
# という実合流形状を作るため --ramps.guess を使う。ramps.guess は MainLane(MergeNode→End) の先頭 200m に
# 加速車線(下側 lane0)を自動付与し、edge を 2つに分割して自動命名する:
#   MainLane-AddedOnRampEdge … 3車線の加速車線部（= 調停対象 mainlane_edge）
#   MainLane                  … 2車線の tail
#   MainLane-AddedOnRampNode  … 加速車線が drop するノード
# これらをクリーンな名前 MergeZone / AccelEnd にリネームする（mainlane_edge=MergeZone）。
set -euo pipefail
cd "$(dirname "$0")"
netconvert \
  --node-files merge.nod.xml \
  --edge-files merge.edg.xml \
  --ramps.guess --ramps.ramp-length 200 \
  --output-file merge.net.xml
# 自動命名 → クリーンな名前（順序重要: Edge/Node の完全一致文字列のみ置換）
sed -i 's/MainLane-AddedOnRampEdge/MergeZone/g; s/MainLane-AddedOnRampNode/AccelEnd/g' merge.net.xml
echo "merge.net.xml generated."
