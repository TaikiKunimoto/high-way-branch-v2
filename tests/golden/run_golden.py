#!/usr/bin/env python3
"""
ゴールデンマスター回帰ハーネス（TraCI リファクタの挙動不変を検証する安全網）

3手法のエントリポイント(default/simple/custom)を固定パラメータでヘッドレス実行し、
決定的な出力（結果CSV・tail CSV）をスナップショットとして採取/比較する。
リファクタ前に `record`、各フェーズ後に `check` して **差分ゼロ** を確認する。

使い方（リポジトリルートから）:
    uv run python tests/golden/run_golden.py record         # 基準採取（リファクタ前）
    uv run python tests/golden/run_golden.py check          # 現状と基準を比較
    uv run python tests/golden/run_golden.py record --fast  # 軽量(300/300)・クラッシュ検出用
    uv run python tests/golden/run_golden.py check  --methods simple,custom

判定:
  - 結果CSV / tail CSV の不一致 = FAIL（挙動が変わった）
  - 正規化stdout の不一致        = WARN（情報。SUMO出力ノイズを含むため参考）

決定性の前提: 各エントリポイントは argv[1] を random.seed に渡し、SUMO config は
speedDev=0.0 で乱数なし。同 (seed, inflow) なら出力は厳密一致するはず。
"""

from __future__ import annotations

import argparse
import glob
import os
import re
import shutil
import subprocess
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
TRACI_DIR = REPO / "TraCI"
SNAP_DIR = Path(__file__).resolve().parent / "snapshots"

# 既定のSUMO_HOME（.pkg framework）。環境変数が優先。
DEFAULT_SUMO_HOME = "/Library/Frameworks/EclipseSUMO.framework/Versions/Current/EclipseSUMO/share/sumo"

# 全手法（congestion を起こす本番相当パラメータ）
FULL_PARAMS = (1700, 1700)
# 軽量（free-flow・クラッシュ検出のみ。協調/混雑パスは網羅しない）
FAST_PARAMS = (300, 300)
ALL_METHODS = ["default", "simple", "custom"]
DEFAULT_SEED = "42"


def sumo_home() -> str:
    # 環境変数を優先するが、bin/sumo が無い（古いbrewパス等）なら framework 既定にフォールバック。
    for sh in (os.environ.get("SUMO_HOME"), DEFAULT_SUMO_HOME):
        if sh and (Path(sh) / "bin" / "sumo").exists():
            return sh
    sys.exit(
        "SUMO not found (no bin/sumo under $SUMO_HOME nor the framework default). "
        "Set SUMO_HOME to your SUMO share/sumo dir."
    )


def key(method: str, seed: str, p: int, e: int) -> str:
    return f"{method}_seed{seed}_p{p}_e{e}"


def normalize_stdout(text: str) -> str:
    """非決定的な行（wall-clock時刻・SUMO性能サマリ）をマスクする。"""
    out = []
    for raw in text.splitlines():
        line = raw.rstrip()
        if line.startswith("Now:"):
            line = "Now: <MASKED>"
        elif line.startswith("Step #"):
            # SUMOの進捗行。タイミング部分(ms/RT/UPS/TraCI)はマスクし、
            # 決定的な車両数(vehicles TOT/ACT/BUF)は残す（挙動の細粒度フィンガープリント）。
            m = re.match(r"^(Step #\d+\.\d+) \(.*?(vehicles TOT \d+ ACT \d+ BUF \d+)\)", line)
            if m:
                line = f"{m.group(1)} (<T> {m.group(2)})"
        elif re.match(r"\s*(Duration|Real time factor|UPS|TraCI-Duration|Performance):", line):
            line = re.sub(r":.*", ": <MASKED>", line)
        out.append(line)
    return "\n".join(out) + "\n"


def result_csv(method: str) -> Path | None:
    pat = str(TRACI_DIR / "simulationStatistics" / "statistics" / method / f"{method}*.csv")
    files = [f for f in glob.glob(pat) if "tail_positions" not in os.path.basename(f)]
    if not files:
        return None
    return Path(max(files, key=os.path.getmtime))


def tail_csv(method: str, seed: str, p: int, e: int) -> Path | None:
    f = TRACI_DIR / "simulationStatistics" / "statistics" / method / f"tail_positions_pass{p}_exit{e}_seed{seed}.csv"
    return f if f.exists() else None


def run_one(method: str, seed: str, p: int, e: int, env: dict[str, str]) -> tuple[str, float]:
    start = time.time()
    proc = subprocess.run(
        [sys.executable, f"{method}.py", seed, str(p), str(e), "--nogui"],
        cwd=str(TRACI_DIR),
        env=env,
        capture_output=True,
        text=True,
    )
    dur = time.time() - start
    if proc.returncode != 0:
        sys.stderr.write(f"[{method}] EXIT {proc.returncode}\n{proc.stderr[-2000:]}\n")
    return proc.stdout, dur


Matrix = list[tuple[str, str, int, int]]


def artifacts_for(method: str, seed: str, p: int, e: int, stdout: str) -> tuple[str, Path | None, Path | None]:
    return normalize_stdout(stdout), result_csv(method), tail_csv(method, seed, p, e)


def do_record(matrix: Matrix, env: dict[str, str]) -> int:
    SNAP_DIR.mkdir(parents=True, exist_ok=True)
    for method, seed, p, e in matrix:
        stdout, dur = run_one(method, seed, p, e, env)
        out_norm, res, tail = artifacts_for(method, seed, p, e, stdout)
        d = SNAP_DIR / key(method, seed, p, e)
        if d.exists():
            shutil.rmtree(d)
        d.mkdir(parents=True)
        (d / "stdout.txt").write_text(out_norm)
        for name, src in (("result.csv", res), ("tail.csv", tail)):
            if src:
                shutil.copyfile(src, d / name)
        print(f"  recorded {key(method, seed, p, e)}  ({dur:.0f}s)")
    print(f"snapshots -> {SNAP_DIR}")
    return 0


def do_check(matrix: Matrix, env: dict[str, str]) -> int:
    failed = False
    for method, seed, p, e in matrix:
        d = SNAP_DIR / key(method, seed, p, e)
        if not d.exists():
            print(f"  SKIP {key(method, seed, p, e)} (no snapshot — run record first)")
            continue
        stdout, dur = run_one(method, seed, p, e, env)
        out_norm, res, tail = artifacts_for(method, seed, p, e, stdout)
        problems = []
        # ハード判定: CSV
        for name, cur in (("result.csv", res), ("tail.csv", tail)):
            golden = d / name
            if golden.exists() and cur:
                if golden.read_bytes() != cur.read_bytes():
                    problems.append(f"FAIL {name} differs")
            elif golden.exists() != bool(cur):
                problems.append(f"FAIL {name} presence mismatch")
        # ソフト判定: stdout
        gold_out = d / "stdout.txt"
        if gold_out.exists() and gold_out.read_text() != out_norm:
            problems.append("WARN stdout differs (informational)")
        status = "OK" if not any(x.startswith("FAIL") for x in problems) else "FAIL"
        if status == "FAIL":
            failed = True
        print(
            f"  [{status}] {key(method, seed, p, e)} ({dur:.0f}s)"
            + ("" if not problems else "  :: " + "; ".join(problems))
        )
    if failed:
        print("\n❌ 挙動が変わっています（FAIL）。差分を確認してください。")
        return 1
    print("\n✅ golden 差分ゼロ（CSV一致）。挙動は保たれています。")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description="TraCI golden-master regression harness")
    ap.add_argument("mode", choices=["record", "check"])
    ap.add_argument("--fast", action="store_true", help="free-flow 300/300（クラッシュ検出のみ・低カバレッジ）")
    ap.add_argument("--methods", default=",".join(ALL_METHODS), help="comma-separated: default,simple,custom")
    ap.add_argument("--seed", default=DEFAULT_SEED)
    args = ap.parse_args()

    p, e = FAST_PARAMS if args.fast else FULL_PARAMS
    methods = [m.strip() for m in args.methods.split(",") if m.strip()]
    matrix = [(m, args.seed, p, e) for m in methods]

    env = {**os.environ, "SUMO_HOME": sumo_home()}
    print(f"SUMO_HOME={env['SUMO_HOME']}")
    print(f"mode={args.mode}  params={p}/{e}  methods={methods}  seed={args.seed}\n")

    return do_record(matrix, env) if args.mode == "record" else do_check(matrix, env)


if __name__ == "__main__":
    raise SystemExit(main())
