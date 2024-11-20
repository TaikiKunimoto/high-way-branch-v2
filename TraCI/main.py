import optparse

import traci
from sumolib import checkBinary


def run():
    while shouldContinueSim():
        traci.simulationStep()
    traci.close()


def startSim():
    """Starts the simulation."""
    traci.start(
        [
            sumoBinary,
            "--net-file",
            "../config/high-way.net.xml",
            "--route-files",
            "../config/high-way.rou.xml",
            "--delay",
            "200",
            "--gui-settings-file",
            "../config/high-way.settings.xml",
            "--start",
        ]
    )


def shouldContinueSim():
    """Checks that the simulation should continue running.
    Returns:
        bool: `True` if there are any vehicles on or waiting to enter the network. `False` otherwise.
    """
    numVehicles = traci.simulation.getMinExpectedNumber()
    return True if numVehicles > 0 else False


def get_options():
    """define options for this script and interpret the command line"""
    optParser = optparse.OptionParser()
    optParser.add_option(
        "--nogui",
        action="store_true",
        default=False,
        help="run the commandline version of sumo",
    )
    options, args = optParser.parse_args()
    return options


if __name__ == "__main__":
    options = get_options()

    # this script has been called from the command line. It will start sumo as a server, then connect and run
    if options.nogui:
        sumoBinary = checkBinary("sumo")
    else:
        sumoBinary = checkBinary("sumo-gui")

    startSim()
    run()
