---
name: no-direct-push-to-main
description: main への直接 push は禁止。変更は必ずブランチを切って commit → PR で出す。
metadata: 
  node_type: memory
  type: feedback
  originSessionId: f7a8cc55-8710-4fc0-b5d4-9e244c73f1a4
---

main ブランチへの直接 push は許可されていない。コード変更・設定変更を問わず、必ずブランチを切って commit し、PR を作成してレビュー経由でマージする。

**Why:** ユーザーが明示的に「main に直 push は許可していない。ブランチを切って PR まで作って」と指示したため。
**How to apply:** 変更を加えるときは最初に `git checkout -b <branch>` でブランチを作成し、commit 後に `git push -u origin <branch>` → `gh pr create` まで行う。main 上で直接 commit / push しない。
