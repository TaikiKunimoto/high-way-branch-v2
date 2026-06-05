import csv
import os


# スクリプトファイルがあるフォルダの絶対パスを取得
current_dir = os.path.dirname(os.path.abspath(__file__))

# seed が 1~10 のファイル名を作る
# 実際には必要に応じて seed の範囲を変更してください
seed_range = range(1, 11)  # 1～10

# ファイルのベース名（"seed" の前まで）
input_filename_base = "tail_positions_pass2520_exit1080_seed"

# 出力ファイル名
output_filename = "custom_tail_positions_3600_30.csv"
# 出力先パスを組み立てる
output_filepath = os.path.join(current_dir, output_filename)

# この辞書に { time値: [各seedのtail_position] } を貯める
time_to_positions = {}

# 実際に読み込む CSV のリストを作る
input_files = [f"{input_filename_base}{seed}.csv" for seed in seed_range]

for seed_num, csv_file in zip(seed_range, input_files):
    # ファイルを開いて読み込む
    # ※ 相対パスの場合は、実行時のカレントディレクトリに注意
    if not os.path.exists(csv_file):
        print(f"ファイルが見つかりません: {csv_file}")
        continue

    with open(csv_file, "r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            # time と tail_position を取り出す
            t = float(row["time"])
            pos = float(row["tail_position"])

            # time_to_positions の中の t に対応するリストを用意
            if t not in time_to_positions:
                # seed の数だけ入るように初期化しておく
                # ただし最初は空でもよいのであとで append もOK
                time_to_positions[t] = {}
            
            # 該当する time に対して、seed の番号を key にして値を格納
            time_to_positions[t][seed_num] = pos

# time の昇順でソートしたい場合が多いので、ソート順のリストを作っておく
sorted_times = sorted(time_to_positions.keys())

# 出力ファイルの列名
# seed の列は seed1, seed2, ... のようにしたい
seed_columns = [f"seed{seed}" for seed in seed_range]
header = ["time"] + seed_columns + ["average"]

with open(output_filepath, "w", encoding="utf-8", newline="") as f_out:
    writer = csv.writer(f_out)
    writer.writerow(header)
    
    for t in sorted_times:
        # この time に対する seed1～seed10 の tail_position を取り出す
        # もし何らかの理由で値が欠損している seed があったら 0 や None など埋める
        positions = []
        for s in seed_range:
            positions.append(time_to_positions[t].get(s, 0.0))
        
        # 平均を計算
        avg_pos = sum(positions) / len(positions)

        # time, seed1,...,seed10, average の順に書き込み
        row = [t] + positions + [avg_pos]
        writer.writerow(row)

print(f"結合ファイルを出力しました: {output_filename}")
