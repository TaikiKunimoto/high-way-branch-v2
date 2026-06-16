#!/usr/bin/env bash
# weave2.net.xml を生成する（両側織込み MD-2）。
#
# 狙う形状: 4車線の織込みゾーン WeaveZone（lane0=加速車線(下,オンランプ由来) / lane1,2=本線 /
# lane3=出口車線(上,オフランプへ)）。合流車は lane0→lane1（下から上）、分流車は本線→lane3（本線から上）で
# 本線を挟んで交差＝両側織込み。
#
# 注: merge/diverge は片側ランプなので ramps.guess で加速/減速車線を自動付与できるが、weave2 は
# オンランプ(下)とオフランプ(上)が対称・逆側にあり、ramps.guess では両方の補助車線を下側(lane0)に置いてしまい
# 「下=加速 / 上=出口」の4車線コアを作れない（加速ゾーン3車線/本線2車線/減速ゾーン3車線に edge 分割もされる）。
# そこで weave2 は手動 connection（weave2.con.xml）で 4車線コアの接続を明示し、意図通りの幾何を得る。
set -euo pipefail
cd "$(dirname "$0")"
netconvert \
  --node-files weave2.nod.xml \
  --edge-files weave2.edg.xml \
  --connection-files weave2.con.xml \
  --output-file weave2.net.xml
echo "weave2.net.xml generated."
