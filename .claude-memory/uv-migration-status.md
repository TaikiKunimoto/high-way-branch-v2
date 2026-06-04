---
name: uv-migration-status
description: "poetry→uv 移行とツールバンプの進捗。コアは PR#2 でmainにマージ済み、後始末が残存。"
metadata: 
  node_type: memory
  type: project
  originSessionId: f7a8cc55-8710-4fc0-b5d4-9e244c73f1a4
---

別デバイスで指示した「poetry→uv 移行 + Python/mypy/ruff バージョンアップ」の進捗（2026-06-05 時点で確認）。

**✅ 完了（PR #2 `e1d1395` で main にマージ済み）**
- poetry→uv: pyproject.toml を PEP621 + `[tool.uv]` に書き換え、poetry.lock 削除、uv.lock 追加、`.python-version`=3.13、README/`.vscode/settings.json` 更新
- Python `requires-python >=3.13`（実機 3.13.5）/ mypy `>=2.1.0`（実2.1.0）/ ruff `>=0.15.0`（実0.15.15）
- `uv lock --check` 同期OK
- traci/sumolib は `[tool.uv] constraint-dependencies` で 1.22 系に固定（SUMO整合のため。1.27化は実機確認後に別途）

**⚠️ 残作業**
1. pre-commit: dev依存に追加済みだが `.pre-commit-config.yaml` が無く未セットアップ
2. ruff 0.15 の新規指摘: lint 7件（B905×2, RUF059×2, B007×2, E712×1）+ `ruff format` 要 3ファイル（custom_cav.py, default.py, status.py）
3. mypy 2.1 strict: 約150件（no-untyped-call 63 / no-untyped-def 43 / var-annotated 16 ほか）。主に simple.py・custom.py・default.py の型注釈不足。simple_cav.py は PR#2 で一部対応済み。※strict は移行前から有効なので大半は既存の未注釈コードで、純粋な移行起因かは要切り分け。
   - root から実行すると import-not-found 12件 → `cd TraCI && mypy .` で解消（実行パスの問題）

**Note:** `.venv` はデバイスごとに `uv sync` が必要（このデバイスは 2026-06-05 に sync 済み）。
