---
name: class-centric-organization
description: コードは操作別の自由関数モジュールでなく、意味を持つclass＋classmethod/staticmethodで構成する
metadata: 
  node_type: memory
  type: feedback
  originSessionId: c5d11bf6-5f18-48e1-b53c-b29cadab3ffe
---

`.py` を「操作の意味ごと」に分けて自由関数を並べるのではなく、**意味を持つ class を定義し、その classmethod / staticmethod として振る舞いを持たせる**ことを好む。例: `Obstacle.from_spec`（パース）/ `Obstacle.validate`（検証）。修論システムモデルの実体（RSU=Layer1調停 / EDF=優先度 / Layer2=実行 / Safety=安全層 / Snapshot / LCRequest）を class にし、生成系は `Snapshot.capture` / `LCRequest.build_all` のような constructor 風 classmethod に寄せる。

**Why:** 既存コードも class 主体（V2CAV / Environment / SimulationStatistics）。実体名が修論システムモデルと 1:1 対応して可読性が上がり、スタイルが統一される。全 staticmethod の class は実質「名前空間」で Python 的には関数のままが慣用との見方もあるが、ユーザーは実体名 class を優先する。
**How to apply:** 新規・リファクタとも自由関数の寄せ集めモジュールより「class＋class/staticmethod」を優先。constructor 系は classmethod（`from_spec` / `capture` / `build_all`）。挙動を変えないリファクタなので決定性（同一 seed の CSV 一致）で確認する。関連: [[raise-on-unexpected-input]]
