#!/usr/bin/env bash
# diverge.net.xml を生成する。本線3車線・1000m(DivergeZone)で分流車を右端lane0へ寄せ(調停)、
# DivergeStart で lane0 が ExitRamp(オフランプ車線~100m)へ分岐。手動con(単純分流, ramps.guess不要)。
set -euo pipefail
cd "$(dirname "$0")"
netconvert --node-files diverge.nod.xml --edge-files diverge.edg.xml --connection-files diverge.con.xml --output-file diverge.net.xml
echo "diverge.net.xml generated."
