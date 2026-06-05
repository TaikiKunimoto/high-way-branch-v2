# Golden-master regression harness

TraCI リファクタの**挙動不変（処理を変えない）**を検証する安全網。
3手法のエントリポイント（`default`/`simple`/`custom`）を固定 `(seed, inflow)` で
ヘッドレス実行し、決定的な出力をスナップショットとして採取/比較する。

## なぜ成立するか（決定性）
- 各エントリポイントは `argv[1]` を `random.seed()` に渡す（同 seed → 同結果）。
- SUMO config は `speedDev=0.0`・`--random` 無し → SUMO 側も決定的。
- 実証: 同条件2回の結果CSVがバイト一致することを確認済み。

## 使い方（リポジトリルートから）
```bash
# リファクタ前に基準を採取
uv run python tests/golden/run_golden.py record

# 各フェーズ後に比較（差分ゼロを確認）
uv run python tests/golden/run_golden.py check

# 軽量(300/300, free-flow)。クラッシュ検出向け。混雑/協調パスは網羅しない
uv run python tests/golden/run_golden.py record --fast
uv run python tests/golden/run_golden.py check  --fast

# 手法を絞る
uv run python tests/golden/run_golden.py check --methods simple,custom
```

## 判定基準
- **結果CSV / tail CSV の不一致 = FAIL**（＝挙動が変わった。マージしない）
- 正規化 stdout の不一致 = 表示のみ（SUMO進捗の `vehicles TOT/ACT/BUF` 等は残し、
  wall-clock タイミングはマスク済み）

## パラメータ
- 既定（本番相当・混雑を起こす）: `1700/1700` … 協調/車線変更/混雑パスを網羅。
- `--fast`: `300/300` … free-flow で速いが低カバレッジ（クラッシュ検出用）。

## SUMO_HOME
SUMO 1.27 を .pkg(framework) で導入。ハーネスは環境変数 `SUMO_HOME` を優先し、
無効（古い brew パス等）なら framework 既定
`/Library/Frameworks/EclipseSUMO.framework/Versions/Current/EclipseSUMO/share/sumo`
へ自動フォールバックする。

## 注意
- `snapshots/` はマシン/SUMOバージョン依存のため **git管理外**（`.gitignore`）。
  リファクタ前に各自 `record` して基準を作る。
- Python client(traci/sumolib) は pin 1.22、SUMO 本体は 1.27。現状この組合せで動作
  （`default_cav` の `lcm=`/`sm=` 非推奨警告は出るが機能する）。
