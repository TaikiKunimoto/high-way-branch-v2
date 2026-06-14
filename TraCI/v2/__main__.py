"""v2 エントリポイント（EDF統一調停・自己完結パッケージ）。

実行: ``cd TraCI && uv run python -m v2 <seed> <inflow> <mlc_ratio> [--env NAME] [--nogui]``
- ``inflow``    : 総流入量 Q [veh/h]
- ``mlc_ratio`` : 必須LC車の比率 f（0..1）
- ``--env``     : 評価環境名（既定 diverge＝分流D）。環境を変えると net・必須LC仕様が切り替わる。

環境（形状）と負荷（Q・f）を分離しており、env を変えるだけで同じ Q,f を別シナリオに適用できる。
結果は simulationStatistics/statistics/v2/ に出力。
"""

import optparse
import os
import random
import sys

from simulationStatistics.simulation_statistics import SimulationStatistics

if "SUMO_HOME" in os.environ:
    tools = os.path.join(os.environ["SUMO_HOME"], "tools")
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

from sumolib import checkBinary
import traci

from v2.environment import ENVIRONMENTS
from v2.obstacle import Obstacle
from v2.simulation import OUTPUT_DIR, V2Simulation

SIMULATION_TIME: float = 600.0  # シミュレーション時間[s]


def _start_sim(sumo_binary: str, sumocfg: str) -> None:
    # --time-to-teleport -1: 全制御を traci で行うため SUMO の jam-teleport を無効化
    # （障害物=停止車両が除去されない／stuck 車は running として残り失敗信号が明確になる）
    traci.start([sumo_binary, "-c", sumocfg, "--time-to-teleport", "-1"])
    print("Simulation started")


def _get_options() -> tuple[optparse.Values, list[str]]:
    parser = optparse.OptionParser(
        usage="python -m v2 <seed> <inflow> <mlc_ratio> [--env NAME] [--obstacle L,P,T] [--nogui]"
    )
    parser.add_option("--env", dest="env", default="diverge", help="evaluation environment name (default: diverge)")
    parser.add_option(
        "--obstacle", dest="obstacle", default=None, help="dynamic obstacle as 'lane,pos,time' (突発障害物)"
    )
    parser.add_option("--nogui", action="store_true", default=False, help="run the commandline version of sumo")
    return parser.parse_args()


def _create_file_name(env_name: str, total_inflow: float, mlc_ratio: float, seed: str) -> str:
    return f"v2_{env_name}_inflow{int(total_inflow)}_mlc{mlc_ratio}_seed{seed}"


if __name__ == "__main__":
    options, positional = _get_options()
    usage = "usage: python -m v2 <seed> <inflow> <mlc_ratio> [--env NAME] [--obstacle L,P,T] [--nogui]"
    if len(positional) < 3:
        sys.exit(f"位置引数が不足しています（必要3: seed inflow mlc_ratio／受け取り {len(positional)} 個）\n{usage}")
    seed = positional[0]  # 乱数シード
    random.seed(seed)
    try:
        total_inflow = float(positional[1])  # 総流入量 Q [veh/h]
        mlc_ratio = float(positional[2])  # 必須LC車の比率 f（0..1）
    except ValueError:
        sys.exit(
            f"inflow と mlc_ratio は数値で指定してください（受け取り: {positional[1]!r}, {positional[2]!r}）\n{usage}"
        )

    env = ENVIRONMENTS.get(options.env)
    if env is None:
        sys.exit(f"不明な --env '{options.env}'（利用可能: {', '.join(ENVIRONMENTS)}）")

    filename = _create_file_name(env.name, total_inflow, mlc_ratio, seed)
    stats = SimulationStatistics(filename=filename, output_dir=OUTPUT_DIR)

    obstacle = Obstacle.from_spec(options.obstacle) if options.obstacle is not None else None

    sumo_binary = checkBinary("sumo" if options.nogui else "sumo-gui")
    _start_sim(sumo_binary, env.sumocfg)
    sim = V2Simulation(
        simulation_time=SIMULATION_TIME,
        env=env,
        total_inflow=total_inflow,
        mlc_ratio=mlc_ratio,
        seed=seed,
        obstacle=obstacle,
    )
    sim.run(stats)
