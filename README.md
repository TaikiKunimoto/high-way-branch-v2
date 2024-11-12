## 実行方法
1. `cd TraCI`  
2. `python main.py`

------------------------------

**各ファイルの開き方**  
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

**その他のファイル**  
・settings.sml：sumo-guiのdelayなどが記述される  
・add.xml：追加の交通オブジェクトや制御情報を記述  

----------------------------

### SUMOメモ  
nodとedgからnetが作られる（neteditを使っていない場合)  
