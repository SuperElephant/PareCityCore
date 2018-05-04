import os, sys
import numpy as np
try:
    sys.path.append(os.path.join(os.path.dirname(
        __file__), '..', 'sumo', 'sumo-master', "tools"))
    from sumolib import checkBinary

except ImportError:
    sys.exit(
        "please declare environment variable 'SUMO_HOME' as the root directory of your sumo installation (it should contain folders 'bin', 'tools' and 'docs')")

import traci
if "../" not in sys.path:
    sys.path.append("../")

from traffic_info.map import *
from traffic_info.car import *
from interface.sumo_interface import *

"""
This is a test main
"""
sumoBinary = checkBinary('sumo')

# without step log
traci.start([sumoBinary, "-c", "../sumo/test_cases/grid_33/grid_33.sumocfg", "--no-step-log", "true"])

# with step log
# traci.start([sumoBinary, "-c", "../sumo/test_cases/grid_33/grid_33.sumocfg"])

sumo_interface = Connection()
# execute the TraCI control loop
step = 0
sumo_interface.extract_map()
while traci.simulation.getMinExpectedNumber() > 0:
    traci.simulationStep()
    step += 1
    sumo_interface.initialize_car()
    sumo_interface.update_travel_time()
    sumo_interface.add_time_step()

    if step == 100:
        """
        TODO: test code here
        active at 100ms
        """

        # test code start============================================


        traffic_map = sumo_interface.map
        cars = sumo_interface.cars
        travel_time = sumo_interface.travel_time_sample

        # test code for edge
        # TODO: there is a problem with sumo_interface
        print("test edge :--------------------------------")
        edge_id = ""
        for id,edge in traffic_map.map.items():  # needs change map
            print("\tid",edge.id)
            edge_id = edge.id
            print("\tupperflow_ids", edge.upperflow_ids)
            print("\tdownflow_ids", edge.downflow_ids)
            print("\tlength", edge.length)
            print("\tset_crosstime({17,})")
            # edge.set_crosstime([17,])
            # print("\tget_crosstime(0)", edge.get_crosstime(0))
            print()

        # example test for car
        print("test car:----------------------------------\n"
              ,cars)

        # example test for travel_time
        print("test travel time-----------------------------:\n"
              ,travel_time)

        # test code end============================================


traci.close()
sys.stdout.flush()