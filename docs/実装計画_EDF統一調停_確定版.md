# 実装計画（確定版）：EDF 統一調停

作成 2026-06-15。**実装済みの `TraCI/v2/` を「確定設計」として記述し直したもの**。
原本 `実装計画_EDF統一調停.md`（計画時点の「現状 custom.py → 改造目標」分析）は履歴として保持する。

本ドキュメントは、計画と実装の相違を**一項目ずつ確定**しながら更新する。各項目には確定状態を付す:
- **確定** … 現行実装を正とし、本書がその仕様を記述する。
- **確定待ち** … 対応方針を未決。下部の「確定待ち項目」に現状を控え、決定後に本文へ昇格する。

確定仕様の一次資料（別フォルダ `修論壁打ち/`）: `優先度アルゴリズム_確定.md` / `コア機構_決定事項.md` / `システムモデル_決定事項.md` / `SUMOデフォルト_LC2013_との差分.md`。

---

## 1. 確定アーキテクチャ 〔確定〕

> 相違#1（インプレース改造 → 新規パッケージ／Layer1・Layer2・RSU 構造）の確定。
> 当初計画は `custom.py`/`custom_cav.py`/`base_cav.py` の改造を想定していたが、**現行の自己完結 v2 パッケージ構成を正式設計とする**。

### 1.1 パッケージ方針

- 提案手法は v1 を改造せず、**自己完結パッケージ `TraCI/v2/` に新規実装**する。
  `v2_cav.py:1-7` が `cav.base_cav`/`cav.custom_cav` から継承・import しないことを明記。物理定数も `constants.py:8-15` で v1 非依存に再定義する（MAX_SPEED/MAX_DECEL/MIN_GAP/REACTION_TIME/FRICTION_COEFFICIENT/TIME_STEP）。
- v1（`TraCI/v1/custom.py`・`default.py`・`simple.py`）は**ベースライン比較用にそのまま保持**し、改造しない。`default.py` ＝ LC2013 ベースライン②。

### 1.2 レイヤ構成

- **Layer1（調停 ＝ RSU/路側機）**
  - `layer1/priority.py` … EDF 鍵。実効距離 `dist` と鍵 `(dist, −wait, −pos, id)`（`EDF.effective_distance` / `make_key` / `order_requests`）。
  - `layer1/rsu.py` … RSU。Phase B の鍵順 提供車割当（`arbitrate`／占有印 `claimed`）、毎Tc 役割付与（`apply_roles`）、検証メトリクス（`keys_unique`／`providers_unique`／`log_assignments`）。
- **Layer2（実行）**
  - `layer2/pair_executor.py` … ペア実行。提供車の協調減速＋安全なら瞬時LC（`execute_pairs`）、同一step同一車線の二重挿入防止（`_slot_free`/`committed`）。
  - `layer2/safety.py` … 安全判定。安全ギャップ `G_req`（`g_req`）と挿入時の前後二方向チェック（`is_insertion_safe`）。
- **観測・状態・縦方向制御** … `v2_cav.py`（`V2CAV`）。自車観測（`update_self_observation`）、活性化（`update_activation`）、縦制御（`control_speed`）、障害物化（`make_obstacle`）。
- **スナップショット S_t** … `snapshot.py`（`Snapshot.capture`／`VehObs`）。Tc 時点の全車観測を凍結し、車線別 縦位置降順インデックス `lane_members` を作る。
- **要求表現・活性化判定** … `lc_request.py`（`LCRequest`／`LCOperation`）。必須LC操作リスト・早め固定活性化窓・待ち時間。
- **環境（シナリオ形状）** … `environment.py`（`Environment`／`Group`／`ENVIRONMENTS`）。net・本線 edge/長・必須LC仕様（`target_lane`/`deadline_pos`）を保持。形状と負荷（Q・f）を分離。
- **突発障害物** … `obstacle.py`（`Obstacle`）。動的配置（`place`）と後方車への回避必須LC付与（`escalate`）。
- **パラメータ** … `constants.py`（物理定数＋機構パラメータ R/TC/DELAY/ACTIVATION_MARGIN）。
- **実行・メインループ** … `simulation.py`（`V2Simulation.run`）。`__main__.py` がエントリ（`<seed> <inflow> <mlc_ratio> [--env] [--obstacle] [--nogui]`）。

### 1.3 RSU の位置づけ

- RSU は**単一スナップショットを 1 回で調停する集中・静的ロジック**（地域分散ではない）。`rsu.py` は状態を持たず staticmethod の集合。
- Layer1/Layer2 の分離は**責務分割（調停 vs 実行）**であって通信分散ではない。「2層分散版（局所化）」は段階4の将来要素として別途検討する（§5）。

### 1.4 メインループ（毎step / 毎Tc 2フェーズ）〔確定〕

> 相違#2 のうち「メインループ新規実装」「Phase A は要求車のみ Key 計算」を確定。
> メインループは `custom.py` を改造せず **`V2Simulation.run`（`simulation.py:108-194`）に新規実装**する。計画の per-veh `decide→execute`（`_decide_yielding_vehicle`/`decide_next_action_and_priority`/`execute_lane_change`）は使わず、EDF/RSU/Layer2 クラスへ委譲する。

`simulation.py:108-194`。各 step:
1. `traci.simulationStep()` ＋ 衝突検出。
2. 到着/未発進/出発処理と**全車観測**（`update_self_observation` → `update_activation`）。スナップショット一貫性のため観測を先に済ませる。
3. **（障害物指定時）`Obstacle.place` ＝ 位置到達トリガで停止車両化、`Obstacle.escalate` ＝ 後方車へ回避必須LC付与。意図的にメインループへ組込んだ確定仕様**（`--obstacle` 指定時のみ作動。突発障害物を毎step監視・動的付与する）。
4. **毎Tc 2フェーズ調停**（`tc_accumulator >= TC`、§1.5 参照）:
   - Phase A … `Snapshot.capture` → `LCRequest.build_all` → `EDF.order_requests`（鍵昇順＝dist小から）。
     **Key 計算の対象は活性化窓内（`D − ACTIVATION_MARGIN` 通過かつ目標車線未達）の要求車のみ**。窓外の車はそもそも必須LC要求を出さないため Key を計算しない（`LCRequest.build_all`／`from_obs` が窓判定で除外）。計画 §2 の「全**要求**車の Key」と一致し、これが確定仕様。
   - Phase B … `RSU.arbitrate`（鍵順に占有印つき提供車確保）→ `RSU.apply_roles`（毎Tc フル再構築：提供車=YIELDING / 要求車=LANE_CHANGING）。
     **譲歩の伝播〔確定〕**: 上位車の提供車として既に `claimed` された要求車は、今Tc は譲る側に回り自分のLCを見送る（`rsu.py:43-46 if req.veh_id in claimed: continue`）。一次資料「コア機構 §4/§6」由来で、本確定版の正式仕様とする。
5. `control_speed`（各車・縦制御）。
6. `Layer2.execute_pairs`（協調減速＋瞬時LC）を最後に呼び、指令が上書きされないようにする。

### 1.5 調停周期 Tc 〔確定（ただし将来変更の余地あり）〕

> 相違#2 のうち「Tc=TIME_STEP」を確定。ただし**後から Tc を step から分離する可能性を残す**。

- 現状 `TC = TIME_STEP = 0.1s`（`constants.py:19`）。`tc_accumulator` による周期ゲート（`simulation.py:165-167`）は実装済みだが、既定では毎 step で成立するため**実質「毎ステップ調停」**として動作する。既定値 0.1s 自体は計画と一致。
- **将来、評価段階で `Tc > TIME_STEP`（調停周期を step から分離）に変更する可能性がある**。その場合、Layer2 実行が直近 Tc のスナップショット／割当を Tc 間で流用する点の妥当性（古いスナップショットでの執行）を再検証すること。現状は Tc=step ゆえ実害なし。

### 1.6 Layer2 の実行時スロット確保（committed）〔仕様確定・要リファクタ〕

> 相違#2 の committed/slot-free を確定。**Layer2 の責務**と位置づける。
> **〔#33/§6 R1 で更新〕** リファクタ R1（`committed` の明示化・幾何を `safety.py` へ集約）は**取り下げ**。#33 が挿入安全を `pair_executor._insertion_safe_live` として Layer2 へ移し `_slot_free`/`committed` と co-locate したため「`safety.py` へ集約」の動機が消えた。`committed`/`_slot_free` の責務（同step・同目標車線の二重挿入防止）自体は #33 後も有効。以下の「要リファクタの理由」は #33 前の文脈。

- **責務の確定**: 同一step・同一目標車線への**位置重なり挿入の防止**は Layer2（実行層）の確定責務。Layer1 の「提供車の確保（`claimed`）」と対称に、Layer2 は「**目標スロットの確保（`committed`）**」を担う。
  - Layer1 ＝ 提供車確保（`rsu.py` `claimed`）… 調停の二重割当防止
  - Layer2 ＝ 目標スロット確保（`pair_executor.py` `committed` / `_slot_free`）… 実行の同時挿入衝突防止
- **Layer1 へ移せない理由**: 要求車が今step 実際に挿入するかは `is_insertion_safe`（gap 充足）次第で実行時にしか確定しない。提供車を得ても gap 未達なら今stepは協調減速に回り挿入しない。スロットが消費されるのは実際の `changeLane` の瞬間だけなので、調停時（Layer1）に予約するのは誤り。加えてスナップショットは Tc 時点で凍結され `is_insertion_safe` は今stepの先行挿入を見られないため、step内の確定を追跡する `committed` は本質的に実行時＝Layer2。

#### 要リファクタの理由（→ §6 R1。コードは後続で修正）

1. **責務の所在が不整合**: 挿入の幾何安全判定は `safety.py`（`is_insertion_safe`）が持つのに、同種の幾何チェックであるスロット重なり判定（`_slot_free`：`VEH_LENGTH + MIN_GAP`）は `pair_executor.py`（実行オーケストレーション）に分散している。挿入安全は「凍結スナップショットに対して」と「今stepの確定挿入に対して」の2方向あり、両方を `safety.py` に集約すべき。
2. **層対称性が実装に現れていない**: Layer1 の `claimed` は `arbitrate` 内の明示的な調停概念だが、Layer2 の `committed` は `execute_pairs` 内のローカル dict に埋もれ、対称な「スロット確保」という設計意図がコードから読み取れない。明示的な名前・型へ昇格すべき。
3. **単体テスト容易性**: §6 検証の「二重割当なし／スロット非重複」を独立に検証するには、スロット確保を独立コンポーネントとして取り出す必要がある。現状はインライン変数のためテスト不能。
4. **Tc≠step 拡張への備え（§1.5）**: Tc を step から分離する場合、Tc 窓内の複数 step にまたがるスロット確保の意味づけ（いつ解放するか）が必要になる。明示コンポーネント化しておくと拡張時に整合を取りやすい。

---

## 2. 確定アルゴリズムの要約

- **優先度＝EDF**。実効距離 `dist = (D − pos) − (k − 1)·R`（D=締切位置、pos=現在縦位置、k=残りLC回数=|現在レーン−目標レーン|、R=1回分の余地）。
- **鍵 ＝ (dist 小, 待ち時間 大, 縦位置〔前方=大〕, ID)**。辞書式・一意 → デッドロックフリー。**種別・酷さは使わない**。
- **執行＝毎 Tc 2フェーズ**。Phase A＝全要求車の Key を同一スナップショットで計算。Phase B＝Key 順（dist小から）に処理し、各要求車に「目標車線後続のうち Key が自分より下位 かつ 今パス未確保 の最近傍」を譲り役に確保（停車中は2番目）。
- **毎 Tc 再割当**。より緊急な車が来れば提供車は自動で付け替わる（**上位優先ゆえ逆転なし**）。`status` は Layer2 の役割表示に降格＝**候補除外に YIELDING を使わない**。
- **G_req ＝ (v×δ) ＋ 制動距離 ＋ minGap**（前後二方向）。**人間の空走(0.75s)を除去**、反応距離は通信遅延 δ（理想 δ=0 でゼロ、段階4で増加）。
- **Θ_force** … 順位ではなく「dist ≤ Θ_force かつ枠なし → 劣化（安全減速で待機）」の挙動閾値。
- **活性化＝早め・固定**（必須LC判明時）。卒論の動的開始位置はコア外（システムモデル §4 改訂）。
- **保証**：デッドロックフリー／二重割当なし／feasible なら全員成功／超過は劣化／衝突は安全層。
- **ベースライン**：①集中版（提案）②SUMOデフォルト LC2013（`default.py`）。

### 2.1 EDF鍵の k・D ＝ アクティブ操作基準 〔確定〕

> 相違#3 を確定。計画は「1台＝単一目標 D」前提で `k=|現在レーン−目標レーン|` としていたが、実装は **1台＝必須LC操作リスト（`operations`）** に一般化し、k・D を**アクティブ操作基準**で測る。これを確定仕様とする。

- 各車は `operations: list[LCOperation]`（`(target_lane, deadline_pos)`）を持つ。`active_operation()` ＝ 未完了のうち **deadline が最も近い操作**（`v2_cav.py:117-122`）。
- 鍵計算の D・k は active_operation 基準: `D = active_operation.deadline_pos`、`k = |現在レーン − active_operation.target_lane|`（`lc_request.py:86,94`）。`dist = (D−pos) − (k−1)·R` は従来どおり。
- **単一操作車（通常の必須LC。lane0→lane2 等の多段LC を含む）は計画と完全一致**: 操作は1つで `target = 最終目標`、`k = |現在 − 最終目標|`。多段でも target は最終目標のまま、レーン変更に応じて k が段階的に減る。
- **差が出るのは複数操作を持つ車 ＝ 現状は障害物回避が append された車のみ**（`obstacle.escalate` / `obstacle.py:104`。spawn 時は最大1操作）:
  - 回避操作（`deadline=障害物位置`, `is_avoidance`）は元の目標より deadline が近いので active になり、D・k が一時的に回避基準へ切り替わる。回避先は隣レーン（`_avoid_lane`=`lane±1`）ゆえ **k=1 → `(k−1)·R=0`**、`dist = 障害物位置 − pos`。
  - 障害物通過で回避操作が `is_done` になり、active_operation が元の最終目標へ**自動復帰**（`lc_request.py:35-39` の `is_avoidance` 完了条件）。
- **確定の根拠**: 障害物回避は最も近い締切で最優先すべきで、アクティブ操作基準の k・D はこの優先順位を自然に表す。最終目標までの残り段数は回避完了後に再評価される。
- **留意（仕様として許容）**: `(k−1)·R` はアクティブ操作内の残り段数にのみ効くため、回避後に残る段（例 lane1→lane2）の余地は回避中には予約しない（回避後に測り直す設計）。

### 2.2 安全ギャップ G_req と挿入判定 〔確定（一部 要修正）〕

> 相違#4 の確定。
> **〔#33 で更新〕** 挿入安全 `is_insertion_safe`（凍結スナップショットの目標車線全車走査）は #33 で**削除**され、`pair_executor._insertion_safe_live`（`getNeighbors` の実時間判定・ジャンクション跨ぎで流入車も捕捉）へ**置換・Layer2 へ移設**された。以下の `safety.py` ベースの記述は #33 前のもので、live 判定として読み替えること（R2 は本変更で吸収、F2 は新式 `_insertion_safe_live`/`_provider_yield` が対象 → §6）。

- **G_req のδ化〔確定〕**: `Safety.g_req = v×δ + 制動距離 + minGap`（`safety.py:19-24`、`DELAY=0` で空走項ゼロ）。計画 T4 通り。
- **挿入判定の必要車間基準を g_req に統一〔確定〕**: 旧v1は追従用 `safety_gap`(0.75s系) を速度差項の基準にしていたが、v2は `g_req`(δ系) を使う（`safety.py:46-55`, `pair_executor.py:74-76`）。**g_req はこの用途のために用意したものなので、これを基準にするのが正**。
- **追従の安全車間がδ化されていない〔要修正 → §6 F1〕**: 縦方向追従の `_calculate_safety_gap` は依然 `reaction = speed × REACTION_TIME(0.75s)`（`v2_cav.py:212`, `constants.py:13`）。δ化は挿入側(G_req)だけで追従側は0.75s系のまま**二系統に分裂**しており、計画 T4「REACTION_TIME 依存を除去」が未達。**追従側もδ化に統一する修正が必要**（後続）。
- **挿入判定の必要車間式の係数〔要検討 → §6 F2〕**: `required = 車長 + MIN_GAP*1.5 + g_req×(speed_diff/MAX_SPEED)`。意図した3項（最低gap＋満額g_req＋速度差gap）と不整合で、minGap 二重計上・絶対/相対速度の扱い等が論点。式・係数は §7 評価で確定。
- **挿入判定の走査範囲〔リファクタ → §6 R2〕**: 現状は目標車線の全車走査（`safety.py:38-58`）だが、**最近傍 follower＋leader の2台判定で衝突安全は十分**（同一縦位置の重なりは leader gap≈0 で捕捉）。最近傍2台へ簡素化を R2 に記録（**衝突0／TTC 検証つき**で確定）。

### 2.3 評価環境（シナリオ）と突発障害物 〔確定〕

> 相違#6 を確定。**環境＝形状（net）／負荷＝パラメータ（総流入Q・必須LC比率f）で分離**し、封鎖B は静的 net でなく**動的突発障害物**で表現する設計を正式とする（`environment.py` / `obstacle.py`）。

**確定する評価環境（`environment.py` の `ENVIRONMENTS`）**:

| 環境 | 名称 | net | 内容 | 締切 D |
|---|---|---|---|---|
| ① D | `diverge` | `config/v2/diverge` | 単一分流（本線3車線、lane2 へ出口） | 2500（出口=本線端） |
| ② M | `merge` | `config/v2/merge` | 単一合流（加速車線 lane0 → lane1） | 200（加速車線端） |
| ③ B素地 | `straight` | `config/v2/straight` | 直進3車線。封鎖Bは `--obstacle` で動的発生 | 障害物位置（動的） |
| ④ MD-1f | `weave` | `config/v2/weave` | **一側**織込み（補助車線 lane0、合流↔分流が逆向き交差・3車線） | 2000 |
| ⑤ MD-2 | `weave2` | `config/v2/weave2` | **両側**織込み（4車線、下=加速車線・上=出口） | 2300 |

- **封鎖B ＝ 動的突発障害物〔確定〕**: B は静的 net を作らず、`straight` 環境＋`--obstacle 'lane,pos,time'` で走行中の1台を停止＝障害物化する（`obstacle.py:55-84`）。例 `--env straight --obstacle 1,1500,80`。形状非依存でどの環境にも付与でき、突発タイミングをパラメータ化できる。**この対応を正式設計とする**。
- **段階の統合〔確定〕**: 計画の「段階2=静的M/B ／ 段階3=動的障害物」を、**B は最初から動的突発障害物**として単一 `Obstacle` 機構に統合する（静的Bは作らない、`obstacle.py`）。
- **織込みは2変種で確定〔確定〕**: 織込み（ウィービング）は **一側 `weave`（環境④ MD-1f）と両側 `weave2`（環境⑤ MD-2）の2環境**が意図された構成（両方とも実 net・rou・sumocfg を整備済み）。計画 §5段階2 が概念として「ウィービング(M+D)」を挙げ、T6 の net 列挙（M/B）に明記が無かっただけ。**2環境とも正式採用**。
- **障害物エスカレーション〔確定〕**: 障害物配置後、後方・同一レーンの車（through も既存MLC車も）に隣レーンへの回避必須LCを動的付与（`obstacle.escalate` / `obstacle.py:86-141`）。回避先は最終目的地側を優先、不可なら逆側退避、通過後に元の目標へ復帰（§2.1 のアクティブ操作切替と連動）。**動的障害物実装に伴う正式仕様とする**。
- **段階4（通信）は未着手**: δ・パケットロス注入・2層分散版は未実装（`DELAY=0.0` のみ）。§4 段階4 の将来課題。

### 2.4 計測（メトリクス）〔一部確定 / テレポートは検討中〕

> 相違#7 の確定。`SimulationStatistics`（v1/v2 共有）で集計し `simulationStatistics/statistics/v2/` に CSV 出力。

**既存の確定メトリクス**: 走行時間、全体平均速度、公平性（追越し回数/index）、TTC（`min_TTC`・TET）、衝突数（`total_collisions`・巻込み台数）、未発進(canceled)台数。

- **締切達成率（必須LC完了率）〔要実装 → §6 F3〕**: 計画 T7 の中核指標。**新規タスクとして実装する**。`is_done`（目標レーン到達）は現状 EDF 調停にしか使われず統計未接続（`v2_cav.py:111-115` の出口統計は走行時間・平均速度のみ）。各必須LC車について「締切までに目標レーンへ到達できたか」を判定し、達成率＝完了数/要求数を CSV 出力する。
- **グループ別平均速度（r_pass/r_exit）〔容認〕**: v2 は `calculate_vehicle_average_speed("", …)`（route="" 固定）で呼ぶため `average_r_pass_speed/average_r_exit_speed` 列は常に None。**現状これらの列は使っていないので容認**（必要になれば route を渡せばよい）。
- **`emergency_brake_counter`〔容認〕**: `v2_cav.py:205` で加算のみ、`SimulationStatistics.increment_emergency_brake` はどこからも呼ばれない。**v1 も同じ**（`base_cav.py:116` で加算のみ・stats 未接続）。v2 固有の劣化ではなく両版共通の未接続。stats 接続（安全メトリクス化）は任意の将来課題。

#### 2.4.1 テレポートの扱い 〔確定（方針A）〕

> 相違#7 のテレポート項目を**方針A**で確定。

- **テレポートは両手法とも無効のまま**（`--time-to-teleport -1`、`__main__.py:38`）。提案は全 traci 制御で SUMO の介入を排し、stuck 車を running として残して失敗を可視化する。
- **失敗の主指標は「締切達成率（§2.4 / §6 F3）＋ 未達・canceled 台数 ＋ 衝突数」に一本化**。テレポート数は LC 破綻の代理指標にすぎず締切達成率で直接計測できるため、**指標から外す**（`getStartingTeleportNumber/getEndingTeleportNumber` は実装しない）。
- ベースライン(LC2013)も**同条件（テレポート無効）で比較**し、失敗は締切未達・stuck として現れる（#8 の同一条件比較とも整合）。
- 文献的な「LC2013＝テレポート破綻」の対比が必要になった場合のみ、ベースラインを SUMO 既定テレポート有効で計数する補助ランを別途追加する（任意）。

### 2.5 活性化（早め固定）〔確定〕

> 相違#9 を確定（想定内）。

- **早め固定活性化を採用**: `D − ACTIVATION_MARGIN` を通過し、かつ目標車線に未達なら活性（`lc_request.py:60-76`）。`update_activation` が操作ごとに一度だけ活性化時刻を記録（`v2_cav.py:124-133`）。卒論の動的開始位置（渋滞末尾）ロジックは v2 に持たない。
- `ACTIVATION_MARGIN = 400.0`（`constants.py:21`）。卒論既定値400mと同値だが、コア外化したのは"動的開始位置ロジック"であって**値の流用は計画違反ではない**。マージン値は §5 の評価対象（暫定値）。

---

## 3. 確定待ち項目（順次、上の本文へ昇格）

> 相違の照合結果（原本との差分）。**各項目は対応方針の指示を受けて確定**し、本文 §1〜§2 や新セクションへ反映する。

| # | テーマ | 現行実装 | 計画(原本) | 状態 |
|---|---|---|---|---|
| ~~2~~ | 調停ループ詳細 | 全項目を §1.4〜§1.6 へ昇格（メインループ新規実装／Tc=step＋将来分離／Phase A要求車のみ／障害物組込み／譲歩の伝播／committed=Layer2責務・要リファクタ）。Layer2毎step実行は Tc=step に従属（§1.5）。 | custom.py 改造・毎Tc執行 | **確定済み** |
| ~~3~~ | EDF鍵の k・D | アクティブ操作基準に一般化（単一操作＝計画一致、複数操作=障害物回避時のみD/k切替）→ §2.1 へ昇格 | 単一目標レーン基準 | **確定済み** |
| ~~4~~ | 安全gap δ化 | §2.2確定（G_reqδ化・required基準g_req統一）／§6 F1（追従側δ化）・F2（必要車間式の3項整理）・R2（最近傍2台へ簡素化・衝突0検証つき） | reaction=v×δ で統一、REACTION_TIME依存除去 | **確定済み** |
| ~~5~~ | Θ_force→`HOLD_MARGIN` | §6 F5：目的を「Dへの滑らかな減速・待機」に限定して実装。定数名 `HOLD_MARGIN` に確定改称。順位には載せない | dist≤Θ_force で劣化（減速待機） | **確定済み（実装はF5）** |
| ~~6~~ | シナリオ/config | 5環境（D/M/B素地/MD-1f/MD-2）＋封鎖B=動的障害物＋エスカレーション＋織込み2変種を §2.3 に確定。段階4通信のみ未着手 | M/B の静的net追加 | **確定済み** |
| ~~7~~ | 計測 | 締切達成率→§6 F3 要実装／グループ別速度・emergency_brake→§2.4 容認／テレポートは方針A（無効のまま・締切達成率で代替）→§2.4.1 確定 | teleport数・締切達成率・TTC・平均速度 | **確定済み** |
| ~~8~~ | ベースライン比較 | 比較戦略＝§7 S1 相談事項（未決）／物理定数差＝§6 F4 要修正（**MAX_ACCEL=2.6 文献根拠**・要感度分析） | 同一net/rou・同一指標で揃える | 相談事項化（S1）＋F4 |
| ~~9~~ | 活性化/ACTIVATION_MARGIN | 早め固定（窓判定）＝§2.5 確定（想定内）。`ACTIVATION_MARGIN=400.0` は §5 評価対象 | 早め固定・マージンは評価決定 | **確定済み** |

### 計画通り（相違なし・確定済みとして据え置き）
- 締切位置 D のシナリオ別定義（出口=2500／加速車線端=200／障害物位置）。
- EDF鍵の符号設計（前方=大・待ち大が上位・ID一意でデッドロックフリー）。
- status の役割表示降格（候補除外に YIELDING 不使用、読むのは `do_not_speed_up` のみ）。
- 活性化を毎step監視・`activated`ガードで一度だけ記録（窓入り捕捉に毎step呼出は必須）。

---

## 4. 段階スライス（実装順）※原本 §5 を保持

- **段階1（最小・分流 net）**：EDF核（2フェーズ・鍵・占有割当・G_req）を分流シナリオで成立。
- **段階2（複合）**：合流M・織込み（M+D）で統一調停を確認。R・Θ_force・G_req係数を評価で詰める。
- **段階3（突発障害物）**：B を動的に発生させ頑健性（実装済み: `obstacle.py`）。
- **段階4（通信）**：δ・パケットロス注入、差分配信の通信量、2層分散版（局所化）で集中版との優位性。

## 5. 未確定パラメータ（評価で決定）※原本 §7 を保持

`R`（既定 50.0）、`HOLD_MARGIN`（旧 `Θ_FORCE`。Dへの滑らかな減速・待機の距離閾値・M/B/D 別 → §6 F5、機構は実装済みだが値は未確定）、`δ`（`DELAY` 既定 0.0）、`G_req` 係数（速度差正規化 → §6 F2）、`Tc`（`TC` 既定 0.1s＝TIME_STEP）、活性化マージン（`ACTIVATION_MARGIN` 既定 400.0）。

> **〔重要・env総改修に伴う再較正〕** 距離系の margin（`ACTIVATION_MARGIN=400`・`R=50`・`HOLD_MARGIN=100`）は**旧ジオメトリ（D≈2500）前提**で設定された。env 再設計（#31〜#45）後の確定 D は **diverge=1000・merge=194・weave=196・weave2=392**。`HOLD_MARGIN=100` は全 D 未満で機能する（F5 で検証済み、§6 F5 参照）が、`ACTIVATION_MARGIN=400` は merge/weave の D を超え区間全体で活性化する。評価フェーズで margin 群を一括再較正すること（絶対値かD比率か、シナリオ別かを含めて決定）。

---

## 6. 残課題の進捗（§6 タスク）

> 当初「仕様確定済み・コード未着手」だった残課題（F=要修正／R=リファクタ）。実装の進行と、別作業 PR #31/#33/#34（merge/diverge 再設計・自力LC）による前提変更を反映して状態を随時更新する。

| # | テーマ | 状態 |
|---|---|---|
| **F1** | 追従の安全車間δ統一 | ✅ 実装済み（PR #27 merged） |
| **F3** | 締切達成率メトリクス | ✅ 実装済み（PR #28 merged） |
| **F4** | 車両物理の統一（MAX_ACCEL=2.6・minGap=2.8） | 🔵 実装中（PR #32） |
| **F5** | 必須LC完走支援: 自己減速で挿入＋D手前 hold で行き止まり防止 | 🔵 実装（PR #48・combined） |
| **F2** | 挿入判定の必要車間式の整理 | 🔁 #33 で対象コードが変化＝再スコープ |
| **R1** | スロット確保(committed)の明示化 | ❌ 取り下げ（#33 で主動機消滅） |
| **R2** | 挿入判定を最近傍2台へ簡素化 | ❌ 取り下げ（#33 が getNeighbors で吸収） |

> **#33（自力LC・getNeighbors 実時間安全判定）による前提変更**: `safety.py` の `is_insertion_safe`（Tcスナップショット全車走査）を削除し、挿入安全を `pair_executor._insertion_safe_live`（getNeighbors 実時間・ジャンクション跨ぎ）へ移設。これにより §1.6/§2.2 の「挿入安全＝凍結スナップショット全車走査」記述は **live 判定**を前提に読み替える必要がある（R1/R2/F2 の前提が変わった）。
> **golden harness**: §6 外だが、v1 エントリ移設で空振り（誤PASS）していた `tests/golden/run_golden.py` を修正済み（PR #30）。

### 要修正（F）

- **F1: 追従の安全車間δ統一** ✅ 実装済み（PR #27 merged）〔§2.2〕
  - `_calculate_safety_gap` を `Safety.g_req`（δ系）へ委譲して二系統分裂を解消し、`REACTION_TIME` を削除。計画 T4「REACTION_TIME 依存を除去」を達成（δ=0 で追従車間が過大になる不公平を解消）。

- **F3: 締切達成率（必須LC完了率）の集計** ✅ 実装済み（PR #28 merged）〔§2.4〕
  - `LCOperation.completed_in_time` / `V2CAV.update_deadline_achievement`・`record_deadline_outcome` を追加し、`SimulationStatistics` に達成率3列（`track_deadline_achievement` フラグで v1 CSV をバイト不変に維持＝golden を壊さない）。母数＝活性化した非回避操作、分子＝締切位置までに目標レーン到達。stuck で running のまま終わった車は失敗計上（§2.4.1 と整合）。障害物化された活性化済み必須LC車も操作を残し失敗計上（`from_obs` が障害物を要求対象外に）。
  - 用途: 提案 vs LC2013 の中核比較指標。実機での値比較は評価（別途）。

- **F2: 挿入判定の必要車間式の整理** 🔁 再スコープ（#33 で対象コードが変化）〔§2.2 / §7〕
  - #33 で旧 `is_insertion_safe`（snapshot 全車走査）が削除され、式が2箇所に移動:
    - `_insertion_safe_live`（getNeighbors）: `required = base(=MIN_GAP*0.5) + (g_req(相手) × Δv/MAX_SPEED if 接近 else 0)`。`dist` は minGap 込みの実ギャップ。
    - `_provider_yield`: `required = 車長 + MIN_GAP*1.5 + (g_req(提供車) × Δv/MAX_SPEED if 接近)`（旧式のまま）。
  - 当初 F2 の論点（g_req の満額/減衰・minGap の二重計上・絶対/相対速度・`/MAX_SPEED` 正規化・係数根拠）は**新式に引き継がれて健在**。2式で base が `MIN_GAP*0.5` と `MIN_GAP*1.5` に割れている点も要整合。
  - 方針: `_insertion_safe_live` と `_provider_yield` の式の一貫性・minGap 計上を点検し整理。具体式・係数は §5/§7 評価で確定（sim 依存）。

- **F4: 提案・ベースラインの車両物理ダイナミクスを統一** 🔵 実装中（PR #32）〔S1 と連動〕
  - **重要な発見（実装時）**: ベースラインも vType 任せではなかった。simple/custom も `setSpeedMode(0)` の traci 制御で、実効加速は `base_cav._calculate_safe_accel_duration`(=Δv/abs(MAX_ACCEL)) → slowDown 経由で **`v1/cav/constants.py:MAX_ACCEL` が決める**（vType accel は speedMode 0 では inert）。よって統一は **v2/constants.py と v1/cav/constants.py の両方の `MAX_ACCEL` を 2.6 に**することで成立（当初案の vType だけでは効かない）。`default_cav` は `MAX_ACCEL=3.0`（Python加速指令なし＝減速のみ）で別件保持。
  - **minGap**: v2 は `setMinGap(2.8)`、v1 simple/custom は `v1/cav/constants.MIN_GAP=2.8`（既に2.8）。vType minGap は `high-way.rou.xml`(v1) では SUMO 挿入/floor に効く（従来2.5→2.8 に統一）。v2 5環境の vType は setMinGap で上書きされ ~inert。
  - 実装済み: v2/constants・v1/cav/constants の MAX_ACCEL=2.6、default_cav デッドコード削除、vType（high-way＋v2各環境）の accel=2.6/minGap=2.8。
  - 残課題: #34(diverge再設計) との `diverge.rou.xml` 衝突解決、vType の範囲確定（high-wayのみ残し v2 inert分を整理するか）、golden full(1700/1700) snapshot の扱い（**新物理で混雑が激化し再採取が非現実的に遅い → 当面 stale を削除＋要再採取注記、fast は維持**）。
  - **MAX_ACCEL の値 ＝ 2.6 m/s²〔確定・文献根拠〕**（許容 2.0〜3.0、感度分析推奨）:
    - 根拠: SUMO 公式 `vClass=passenger` 既定 `accel=2.6`（公式が「物理最大でなく convenient/快適値」と明記）。IDM 現実レンジ 0.8〜2.5（Treiber & Kesting 公式 "Realistic values are 0.8 to 2.5 m/s²"）、IDM原典 a=0.73、CAV/ACC 研究の制御上限 +2〜+3。現状 10.0 は物理限界（タイヤ摩擦 6.9〜9.8）すら超える非現実値。
    - 実装注意: 提案は traci 制御なので **`MAX_ACCEL` を Python 側で 2.6 に明示クリップ**する必要（vType の accel 設定だけでは traci 直接指令に効かない）。ベースライン(SUMO/LC2013制御)と提案(traci)で**有効上限が同一**になっているか双方で検証。
    - 併せて確認: decel は SUMO既定 4.5（快適）/ emergencyDecel 9 に対し v2 は `MAX_DECEL=-5.0`。加速だけ揃えて減速前提がずれないか。**2.0 / 2.6 / 3.0 の感度分析を一度実施**（指標悪化が「実力」か「上限低下の副作用」かを切り分け）。
    - 出典: SUMO Vehicle Type Parameter Defaults / Treiber–Hennecke–Helbing 2000 / traffic-simulation.de / MDPI Future Transportation 2026 ほか（調査ログ `tasks/w8je7ow3e`）。

- **F5: 必須LC完走支援（自己減速で挿入＋D手前 hold で行き止まり防止）** 🔵 実装（PR #48・combined）。env 総改修（#31〜#45）完了で保留解除して再開（旧称 Θ_force）〔旧 相違#5〕
  - **調査で判明した基本アルゴリズムの問題**: 要求車が**最高速のまま一切減速しない**ため、低速・混雑した目標レーンへ挿入できない（`_insertion_safe_live` は速度差比例で必要ギャップが増えるので、物理ギャップがあっても「速すぎて入れない」）。協調（提供車）は**後方車にしか頼めず**（`_find_provider` は `lane_pos < pos` のみ）、後方に開くギャップに要求車自身が落ちて入ることもできない。weave の2段分流(lane2→1→0)で特に顕在化し挿入不可のまま末端で急停止（veh159 を逐次トレースで機序確定）。
  - **機構①（挿入の成立）self-decel** `pair_executor._requester_match_target_speed`: safe gap が無い要求車が、目標レーン先行の速度まで**自分も減速**して速度差を縮め、既存ギャップへ滑り込む。提供車の後方ギャップ開けと相補的。
  - **機構②（最終手段の安全網）F5 hold** `_hold_before_deadline`: 締切間近 `dist = EDF.effective_distance(req) ≤ HOLD_MARGIN` でなお挿入不可なら、D 手前で滑らかに停止し**行き止まり(lane-drop)の急停止を防ぐ**＋D で待って入る猶予を作る。`HOLD_MARGIN` を `constants.py` に追加（暫定100m・評価で確定）。EDF 鍵には載せない。
  - **leader 尊重（旧 保留理由①を解消）**: 両機構とも own leader が `safety_gap` 内ならスキップし control_speed に委ねる（弱い減速指令が `setSpeedMode(0)` 下で緊急ブレーキを上書きし、減速/hold 車列で後続が追突するのを防ぐ）。
  - **検証（v2 smoke 全5環境 Q1200/f0.3・逐次/決定的・seed42、F5-off=main #45 と同一物理で比較）**: combined は**全環境で締切100%・行き止まり急停止0・衝突0**（merge 56→59/e18→0・weave 56→59/e51→0・weave2 55→57/e6→0・diverge 37/37・straight 0/0）。内訳: F5 単独で merge/weave2 を満点化、self-decel 追加で weave(2段分流)も59/59。**測定は逐次**（並列バッチは SUMO/traci 同時起動で稀に乱れるため不採用）。
  - **残課題（評価で確定）**: `HOLD_MARGIN`・self-decel 目標速度は単一 seed/負荷での確認。複数 seed・負荷掃引での頑健性確認は評価フェーズ。（旧 保留理由②の「`HOLD_MARGIN`=100 > D で全区間発火」は新ジオメトリ＝diverge **D=1000**/merge D=194/weave D=196/weave2 D=392 がいずれも 100 超で解消、かつ combined で吸収）。

### リファクタ（R）— いずれも #33 で取り下げ

- **R2: 挿入判定を最近傍2台へ簡素化** ❌ 取り下げ（#33 が吸収）〔§2.2〕
  - #33 で旧 `is_insertion_safe`（目標車線の全車走査）が削除され、`pair_executor._insertion_safe_live` が `getNeighbors` で隣接後続/前走のみを実測＝**本質的に近傍判定**に。R2 の目的（O(n)→最近傍2台）は別手段で達成済みのため取り下げ。

- **R1: Layer2 のスロット確保（committed/slot-free）の明示化** ❌ 取り下げ（#33 で主動機消滅）〔§1.6〕
  - 当初狙い「挿入安全(`is_insertion_safe`)とスロット(`_slot_free`)が2箇所に分散→`safety.py` に集約」は、#33 が `is_insertion_safe` を削除し挿入安全(`_insertion_safe_live`)とスロット(`_slot_free`)を**両方 `pair_executor` に co-locate** したことで消滅。残るのは `committed` の明示化（テスト容易性・対称性）のみで費用対効果が低く取り下げ。
  - 補足: 両側→中央 同スロット二重挿入の防止は #33 後も `committed`/`_slot_free` が担う（`target_lane` キーで源レーン非依存。§1.6 の「committed が凍結スナップショットの見落としを埋める」性質は #33 の live 判定下でも有効）。

---

## 7. 相談事項（未決）

> 確定でも残課題（実装）でもなく、**研究方針として未決**のもの。決まり次第、確定 or タスク化する。

- **S1: ベースライン比較戦略（何を比較対象＝"敵"にするか）**〔相違#8〕
  - 状況: 提案を**何と比較するか未決**。LC2013（`default.py`）が候補だが、現状そのままでは同一指標で比較できない（テレポート＝方針Aで不使用／締切達成率＝F3で実装予定／出力先 `statistics/default` vs `statistics/v2`／テレポートフラグ非対称）。
  - 確認済みの事実: **net/rou は v1 `high-way` ≡ v2 `diverge`（バイト一致）**。活性化位置も diverge `D=2500−400=2100` が default の LC2013 有効化ゲート `MERGE_STAET_POS=2100` と縦位置一致。**相違の本質は net でなく「機構（SUMO LC2013 vs EDF調停）と指標非統一」**。
  - 保留理由: 比較対象（LC2013 単独か、他手法も含むか等）が未確定。決まり次第、指標統一（締切達成率・衝突・平均速度・TTC を両手法で揃える／出力先・テレポート条件を統一）を F タスク化する。物理ダイナミクス統一は §6 F4 で先行。
