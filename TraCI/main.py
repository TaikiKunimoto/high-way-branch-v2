import traci
from sumolib import checkBinary


sumoBinary = checkBinary('sumo-gui')

def main():
    startSim()

    while shouldContinueSim():
        traci.simulationStep()
    traci.close()

def startSim():
    """Starts the simulation."""
    traci.start(
        [
            sumoBinary,
            '--net-file', '../high-way.net.xml',
            '--route-files', '../high-way.rou.xml',
            '--delay', '200',
            '--gui-settings-file', '../high-way.settings.xml',
            '--start'
        ])

def shouldContinueSim():
    """Checks that the simulation should continue running.
    Returns:
        bool: `True` if there are any vehicles on or waiting to enter the network. `False` otherwise.
    """
    numVehicles = traci.simulation.getMinExpectedNumber()
    return True if numVehicles > 0 else False

if __name__ == "__main__":
    main()