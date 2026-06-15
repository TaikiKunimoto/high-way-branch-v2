#!/usr/bin/env bash
# diverge.net.xml を生成する。3本線 + 下側に専用減速車線(off-ramp lane) ~600m。
# ramps.guess が off-ramp の減速車線(下側 lane0)を MainApproach 末尾 600m に付与し edge を分割・自動命名するので、
# MainApproach-AddedOffRampEdge -> DivergeZone, MainApproach-AddedOffRampNode -> DivergeStart にリネーム。
set -euo pipefail
cd "$(dirname "$0")"
netconvert --node-files diverge.nod.xml --edge-files diverge.edg.xml \
  --ramps.guess --ramps.ramp-length 600 --output-file diverge.net.xml
sed -i 's/MainApproach-AddedOffRampEdge/DivergeZone/g; s/MainApproach-AddedOffRampNode/DivergeStart/g' diverge.net.xml
echo "diverge.net.xml generated."
