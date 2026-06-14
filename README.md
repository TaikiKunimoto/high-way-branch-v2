## 環境構築
uv を利用して依存環境をインストールします．本プロジェクトは **Python 3.13** が必要です．
1. uv をインストール：`curl -LsSf https://astral.sh/uv/install.sh | sh`  
2. Python 3.13 を用意する（例：`uv python install 3.13`。リポジトリの `.python-version` で 3.13 を指定済み）  
3. `uv sync`（仮想環境 `.venv` を Python 3.13 で作成し，依存をインストール）  
4. `cmd + p` で workspace と検索し，`Open Workspace` をクリックしてワークスペース内で開発を行う

------------------------------

## 実行方法
実行ファイル，乱数seed値，交通量（流入数）を指定しシミュレーションを実行します．
引数は順に `乱数seed値 inflow_pass inflow_exit` です．

1. `cd TraCI`  
2. 実行したい手法に応じて以下のいずれかを実行します．  
手法は `TraCI/v1/`（ベースライン）と `TraCI/v2/`（EDF統一調停・新規）に分かれています。  
   - デフォルト手法（旧 `main.py`）： `uv run python -m v1.default 1 1700 1700`  
   - シンプルな車線変更手法： `uv run python -m v1.simple 1 1700 1700`  
   - カスタム手法（卒論提案）： `uv run python -m v1.custom 1 1700 1700`  
   - v2手法（EDF統一調停・新規）： `uv run python -m v2 <seed> <inflow> <mlc_ratio> [--env NAME] [--obstacle L,P,T]`  
     例：`uv run python -m v2 1 3400 0.5`（総流入3400・必須LC比率0.5・既定env=diverge）。

     **評価環境（形状）**は `--env` で切替：`diverge`①／`merge`②／`straight`（素地）／`weave`④(MD-1f)／`weave2`⑤(MD-2)。  
     **突発障害物（パラメータ）**は `--obstacle lane,pos,time` で任意の環境に付与（走行中の1台を停止＝障害物化）。例：  
     - 環境③ S-B1（単一障害物）： `--env straight --obstacle 1,1500,80`  
     - 環境⑥ MB-b（合流+封鎖）： `--env merge --obstacle 2,100,20`  
     - 環境⑦ DB-c（分流+封鎖）： `--env diverge --obstacle 1,1500,80`

> 設定ファイルは分離： `config/v1/`（ベースライン high-way）／`config/v2/<env>/`（環境ごと）。

------------------------------

**各ファイルの開き方**  
`cd config`  
`netedit filename.net.xml`  
`sumo-gui filename.sumocfg`  

**1. neteditの編集**  
- nodeやedgeの保存： File -> Save Network (`Ctrl + S`)  
※file name : helloWorld.net.xml  

- 車両情報,ルートの保存(Demand)： File -> Demand elements -> Save demand elements (`Ctrl + Shift + D`)  
※file name : helloWorld.rou.xml  

**2. neteditからSUMO-GUIで可視化**    
Edit -> Open in sumo-gui (`Ctrl + T`)  
SUMO-GUIの保存：File -> Save Configuration (`Ctrl + Shift + S`)  
※file name : helloWorld.sumocfg  

**3. SUMO-GUIのview-settingsの変更**    
![image](https://github.com/user-attachments/assets/1db714ee-b667-4072-aad9-39094a5c179a)
※file name : helloWorld.view.xml  

**その他のファイル**  
・settings.sml：sumo-guiのdelayなどが記述される  
・add.xml：追加の交通オブジェクトや制御情報を記述  

----------------------------

### SUMOメモ  
nodとedgからnetが作られる（neteditを使っていない場合)  
