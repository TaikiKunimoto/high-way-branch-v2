"""v2 エントリポイント（EDF統一調停・自己完結パッケージ）。

実行: ``cd TraCI && uv run python -m v2 <seed> <inflow_pass> <inflow_exit> [--nogui]``
（v2 一式は TraCI/v2/ パッケージに集約。本ファイルは ``python -m v2`` で実行される __main__）。
引数規約は既存の custom.py / default.py と同じ。結果は simulationStatistics/statistics/v2/ に出力。
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

from v2.simulation_state import OUTPUT_DIR, V2SimulationState, run

SIMULATION_TIME: float = 600.0  # シミュレーション時間[s]


def _start_sim(sumo_binary: str) -> None:
    traci.start([sumo_binary, "-c", "../config/high-way.sumocfg"])
    print("Simulation started")


def _get_options() -> optparse.Values:
    parser = optparse.OptionParser()
    parser.add_option(
        "--nogui",
        action="store_true",
        default=False,
        help="run the commandline version of sumo",
    )
    options, _ = parser.parse_args()
    return options


def _create_file_name(inflow_pass: int, inflow_exit: int, seed: str) -> str:
    return f"v2_pass{inflow_pass}_exit{inflow_exit}_seed{seed}"


if __name__ == "__main__":
    options = _get_options()
    args = sys.argv
    seed = args[1]  # 乱数シード
    random.seed(seed)
    inflow_pass = int(args[2])  # pass側車両流入数
    inflow_exit = int(args[3])  # exit側車両流入数

    filename = _create_file_name(inflow_pass, inflow_exit, seed)
    stats = SimulationStatistics(filename=filename, output_dir=OUTPUT_DIR)

    sumo_binary = checkBinary("sumo" if options.nogui else "sumo-gui")
    _start_sim(sumo_binary)
    state = V2SimulationState(SIMULATION_TIME)
    run(state, inflow_pass, inflow_exit, stats, seed)
