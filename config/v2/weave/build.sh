#!/usr/bin/env bash
# weave.net.xml を生成する。
#
# 一側織込み(MD-1f): 本線2車線の手前独立区間 → オンランプとオフランプを近接配置した織込み区間 → 本線2車線。
# ramps.guess がオンランプの加速車線とオフランプの減速車線を 1本の連続した補助車線(下側 lane0)に統合し、
# WeaveZone を 3車線(lane0=補助/下, lane1,2=本線) にする（両ランプが WeaveZone 全長をカバーするため edge 分割は起きない）。
# 合流車は OnRamp→補助車線(lane0)→本線(lane1)、分流車は 本線→補助車線(lane0)→ExitRamp で、補助車線で逆向きに交差＝織込み。
set -euo pipefail
cd "$(dirname "$0")"
netconvert \
  --node-files weave.nod.xml \
  --edge-files weave.edg.xml \
  --ramps.guess --ramps.ramp-length 200 \
  --output-file weave.net.xml
echo "weave.net.xml generated."
