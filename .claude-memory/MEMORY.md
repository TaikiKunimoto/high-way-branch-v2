# Memory Index

Claude Code の永続メモリ。このディレクトリは git 管理され、
`~/.claude-personal/projects/<slug>/memory` からシンボリックリンクで参照される。

別デバイスでセットアップする場合は `scripts/link-memory.sh` を実行する。

<!-- 1行1メモリ: - [Title](file.md) — hook -->

- [main直push禁止](no-direct-push-to-main.md) — 変更は必ずブランチ→commit→PR
- [uv移行の進捗](uv-migration-status.md) — poetry→uv済(PR#2)、pre-commit/ruff/mypy後始末が残存
- [想定外入力は即raise](raise-on-unexpected-input.md) — 境界で検証し受け取った値つきで明示エラー
- [class主体の構成](class-centric-organization.md) — 操作別関数モジュールでなく意味を持つclass＋class/staticmethod
