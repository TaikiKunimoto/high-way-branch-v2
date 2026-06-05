---
name: uv-migration-status
description: poetry→uv 移行とツールバンプの進捗。コアはマージ済み、後始末3件はPR
metadata: 
  node_type: memory
  type: project
  originSessionId: f7a8cc55-8710-4fc0-b5d4-9e244c73f1a4
---

別デバイスで指示した「poetry→uv 移行 + Python/mypy/ruff バージョンアップ」の進捗（2026-06-05 更新）。

**✅ 完了・main マージ済み（PR #2 `e1d1395`）**
- poetry→uv: pyproject を PEP621+`[tool.uv]` 化、poetry.lock削除、uv.lock追加、`.python-version`=3.13、README/.vscode 更新
- Python `>=3.13`(実3.13.5) / mypy `>=2.1.0`(実2.1.0) / ruff `>=0.15.0`(実0.15.15)
- traci/sumolib は `constraint-dependencies` で 1.22 系固定（SUMO整合、1.27化は実機確認後に別途）

**🔵 残3件 → 実装済み・レビュー待ち（スタックPR、#4→#5→#6 の順でマージ）**
- PR #4 `chore/ruff-cleanup`(base main): ruff 0.15 の lint 7件 + format
- PR #5 `chore/mypy-strict`(base #4): mypy 2.1 strict 142件解消。設定に mypy_path/files=TraCI 追加、sumolib/matplotlib を ignore に。**潜在バグ修正: simple.py の `calculate_vehicle_average_spped`(タイプミス、実行時AttributeError)→`...speed` 4箇所**。simple_cav の None演算は custom_cav に倣ってガード（挙動不変）。
- PR #6 `chore/pre-commit`(base #5): `.pre-commit-config.yaml`（ruff/mypy を local フック）。`uv run pre-commit install` で有効化。

**メモ / 既知の癖**
- `.gitignore` の `statistics` が広すぎて `simulationStatistics/statistics/` のソース(calculate_avrage_tail_pos.py ×2)まで巻き込み、`ruff check .` がこれらを素通りしていた（pre-commit は明示パスで検査するので検出・修正済み）。パターン具体化は未対応。
- `.venv` はデバイス毎に `uv sync` が必要。mypy はメソッド内の代入で型がナローイングされるため、IDE が出す simple_cav の型エラー(282/288 等)は古いキャッシュ由来で CLI `uv run mypy` は0件が正。
