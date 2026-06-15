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

> 相違#2 の committed/slot-free を確定。**Layer2 の責務**と位置づける。仕様は確定だが**現状の実装形態は要リファクタ**（理由は下記）。**コード修正は後続タスクとし、本確定版では未着手**。

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

`R`（既定 50.0）、`HOLD_MARGIN`（旧 `Θ_FORCE`。Dへの滑らかな減速・待機の距離閾値・M/B/D 別・**未実装** → §6 F5）、`δ`（`DELAY` 既定 0.0）、`G_req` 係数（`1.5×minGap`・速度差正規化 → §6 F2）、`Tc`（`TC` 既定 0.1s＝TIME_STEP）、活性化マージン（`ACTIVATION_MARGIN` 既定 400.0）。

---

## 6. 残課題（仕様確定済み・コード未着手）

> **現時点では実装しない**。F=要修正（計画達成のために挙動を直す必要）／R=リファクタ（挙動は妥当だが構造を改善）。

### 要修正（F）

- **F1: 追従の安全車間 `_calculate_safety_gap` をδ化に統一**〔§2.2〕
  - 現状: `v2_cav.py:212 reaction_distance = self.speed * REACTION_TIME(0.75s)`。挿入側 `g_req` はδ化済みだが追従側は0.75s系のまま二系統に分裂。
  - 方針: 追従側の空走項も `speed × DELAY`（δ系）へ統一し、`REACTION_TIME` 依存を除去（`constants.py:13` の扱いも整理）。計画 T4 の達成。
  - 理由: 計画 T4「REACTION_TIME 依存を除去」が未達。δ=0（理想通信）で追従車間が過大になり、提案手法の評価が不公平になりうる。

- **F3: 締切達成率（必須LC完了率）の集計を実装**〔§2.4〕
  - 現状: `is_done`（目標到達）は EDF 調停専用で統計未接続。締切達成率の集計が無い（計画 T7 の中核指標）。
  - 方針: 各必須LC車の「締切までに目標レーンへ到達したか」を判定し、達成率＝完了数/要求数 を CSV 出力。出口統計 `accumulate_exit_stats`（`v2_cav.py:111-115`）に到達判定を追加し、`SimulationStatistics` に達成率フィールド・列を追加する。
  - 用途: 提案 vs LC2013 ベースラインの中核比較指標（§6 検証「feasible なら全員成功」）。テレポート方針（§2.4.1）と合わせ、失敗の主指標に据える。

- **F2: 挿入判定の必要車間式の見直し（`1.5×minGap`・`speed_diff/MAX_SPEED`・g_req の合成）**〔§2.2 / §7〕
  - 現状: `required = 車長 + MIN_GAP*1.5 +（speed_diff>0 のとき）g_req(相手speed) × (speed_diff/MAX_SPEED)`（`safety.py:46-55`, `pair_executor.py:74-76`）。
  - 意図したモデル: 必要車間 ＝「最低限のgap」＋「安全用 g_req（満額）」＋「速度差を考慮したgap」の3項構成。
  - 不整合・論点:
    1. 現状は g_req を満額足さず `speed_diff/MAX_SPEED` で減衰させ、安全項と速度差項が混在している（意図の3項に分かれていない）。
    2. g_req は既に `minGap ＋ 制動距離(絶対速度)` を含むため、`MIN_GAP*1.5` の加算は **minGap の二重計上**。
    3. 挿入の gap 受容で本来効くのは絶対速度でなく**相対（接近）速度**。`g_req(絶対速度)` を基準にする妥当性が要検討。
    4. `1.5` 倍・`/MAX_SPEED` 正規化はヒューリスティックで根拠が弱い（§7 の評価対象）。
  - 方針: 3項構成（最低gap ＋ 満額 g_req ＋ 相対速度 gap）へ整理し二重計上を排除。具体式・係数は §5 評価で確定。

- **F4: 提案・ベースラインの車両物理ダイナミクスを統一**〔§8 S1 と連動〕
  - 現状: 提案 `MAX_ACCEL=10.0`（`v2/constants.py:10`、"過大"注記）を Python 制御で実効。ベースライン `default_cav` は実ダイナミクスを SUMO vType に委ね（`MAX_ACCEL=3.0` は観測値クリップ用 `default_cav.py:113`）、`MIN_GAP=2.5` はデッドコード（`setMinGap` 未呼び出し・`safety_gap` 未使用）。`MAX_DECEL` は両者 -5.0 で一致。
  - 方針: 車両物理（最大加速・最大減速・最小車間・車長）を提案・ベースラインで**完全統一**し、**違いを LC 決定ロジックだけにする**。過大な `MAX_ACCEL=10.0` を現実的な共通値へ下げ、提案の Python 制御とベースラインの vType を一致させる。`MIN_GAP` は 2.8 に統一（`default_cav` のデッドコード 2.5 は削除、vType minGap も 2.8）。
  - **MAX_ACCEL の値 ＝ 2.6 m/s²〔確定・文献根拠〕**（許容 2.0〜3.0、感度分析推奨）:
    - 根拠: SUMO 公式 `vClass=passenger` 既定 `accel=2.6`（公式が「物理最大でなく convenient/快適値」と明記）。IDM 現実レンジ 0.8〜2.5（Treiber & Kesting 公式 "Realistic values are 0.8 to 2.5 m/s²"）、IDM原典 a=0.73、CAV/ACC 研究の制御上限 +2〜+3。現状 10.0 は物理限界（タイヤ摩擦 6.9〜9.8）すら超える非現実値。
    - 実装注意: 提案は traci 制御なので **`MAX_ACCEL` を Python 側で 2.6 に明示クリップ**する必要（vType の accel 設定だけでは traci 直接指令に効かない）。ベースライン(SUMO/LC2013制御)と提案(traci)で**有効上限が同一**になっているか双方で検証。
    - 併せて確認: decel は SUMO既定 4.5（快適）/ emergencyDecel 9 に対し v2 は `MAX_DECEL=-5.0`。加速だけ揃えて減速前提がずれないか。**2.0 / 2.6 / 3.0 の感度分析を一度実施**（指標悪化が「実力」か「上限低下の副作用」かを切り分け）。
    - 出典: SUMO Vehicle Type Parameter Defaults / Treiber–Hennecke–Helbing 2000 / traffic-simulation.de / MDPI Future Transportation 2026 ほか（調査ログ `tasks/w8je7ow3e`）。

- **F5: 枠なし要求車の「Dへ向けた滑らかな減速・待機」挙動を実装**（旧称 Θ_force を**目的限定＋改称**）〔旧 相違#5〕
  - **目的の限定**: 衝突は `control_speed`＋緊急減速、Dの超過防止は SUMO トポロジー（teleport 無効で lane-drop/分岐手前に stuck）が既に担保。よって本挙動の役割は**「枠が取れない要求車が分岐直前まで巡航 → SUMO トポロジーで急停止」になるのを避け、D の手前で滑らかに減速して待つ**という**挙動品質**に限定する（劣化/強制ではない）。
  - 挙動: 今Tc 割当なし かつ 実効距離 `dist ≤ <閾値>` の要求車を、D の手前で安全に減速・保持する（提供車の協調減速とは別の、**要求車自身の committed-wait**）。後続Tc で枠が開けば通常どおり挿入。**EDF の順位（鍵）には載せない**（順序は dist が決める設計を維持）。
  - **定数名 ＝ `HOLD_MARGIN`〔確定〕**: 旧 `Θ_FORCE`/`THETA_FORCE` は "force/劣化" の含意が実態（滑らかな減速）と合わないため改称。`ACTIVATION_MARGIN` と対の「D からの距離マージン」命名（枠が無いとき D の手前で hold する余地）。
  - 値: シナリオ別（M/B/D）の暫定値（§5 評価対象）。
  - 検証: 急停止が消え後続衝突・異常が出ないこと、締切達成率（F3）が改善 or 不変。

- **R2: 挿入判定の走査を「最近傍 follower ＋ 最近傍 leader の2台」へ簡素化**〔§2.2〕
  - 現状: `is_insertion_safe` が目標車線の**全車を走査**（`safety.py:38-58`）。
  - 方針: 最近傍の後続1台＋先行1台のみの判定に戻す（v1 方式、O(n)→O(1)）。同一縦位置の重なりは leader 側 gap≈0 で従来どおり捕捉。
  - 理由: 衝突安全は最近傍2台で決まり、同一縦位置の側面衝突も最近傍 leader（gap≈0 < required）が捕える。全車走査が追加で守る「最近傍でない高速車」ケースは車間追従（`control_speed`）下では速度が揃い起きにくく、最終的に安全層（緊急減速）が担保する。
  - **検証条件（必須）**: 簡素化後に **衝突0／TTC（`min_TTC`・TET）が悪化しないこと**を評価で確認してから確定（安全に直結するため）。

- **R1: Layer2 のスロット確保（committed/slot-free）の明示化**〔§1.6〕
  - 現状: `pair_executor.py:41,60-63` の `committed` dict ＋ `_slot_free` がインラインで `execute_pairs` 内に埋もれている。
  - 方針: 幾何判定（`VEH_LENGTH + MIN_GAP` のスロット重なり）を `safety.py` へ移し、`Safety.is_insertion_safe` と同じ「挿入安全」の家族として集約。スロット確保状態は Layer1 の `claimed` と対称な明示レジストリとして Layer2 に保持。`pair_executor.py` は「割当ループ＋安全問い合わせ＋`changeLane`/`slowDown` 発行」のオーケストレーションに専念。
  - 理由: ①責務の所在が不整合（挿入安全が2箇所に分散）／②層対称性が実装に現れていない／③単体テスト容易性（§6検証の独立検証）／④Tc≠step 拡張（§1.5）時のスロット解放意味づけ。

---

## 7. 相談事項（未決）

> 確定でも残課題（実装）でもなく、**研究方針として未決**のもの。決まり次第、確定 or タスク化する。

- **S1: ベースライン比較戦略（何を比較対象＝"敵"にするか）**〔相違#8〕
  - 状況: 提案を**何と比較するか未決**。LC2013（`default.py`）が候補だが、現状そのままでは同一指標で比較できない（テレポート＝方針Aで不使用／締切達成率＝F3で実装予定／出力先 `statistics/default` vs `statistics/v2`／テレポートフラグ非対称）。
  - 確認済みの事実: **net/rou は v1 `high-way` ≡ v2 `diverge`（バイト一致）**。活性化位置も diverge `D=2500−400=2100` が default の LC2013 有効化ゲート `MERGE_STAET_POS=2100` と縦位置一致。**相違の本質は net でなく「機構（SUMO LC2013 vs EDF調停）と指標非統一」**。
  - 保留理由: 比較対象（LC2013 単独か、他手法も含むか等）が未確定。決まり次第、指標統一（締切達成率・衝突・平均速度・TTC を両手法で揃える／出力先・テレポート条件を統一）を F タスク化する。物理ダイナミクス統一は §6 F4 で先行。
