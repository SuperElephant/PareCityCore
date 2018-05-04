import os
import sys
import numpy as np

try:
    sys.path.append(os.path.join(os.path.dirname(
        __file__), '..', 'sumo-master', "tools"))
    from sumolib import checkBinary
except ImportError:
    sys.exit(
        "please declare environment variable 'SUMO_HOME' as the root directory of your sumo installation (it should contain folders 'bin', 'tools' and 'docs')")

import traci


class SumoGraph:
    def __init__(self):
        self.edge_map = {}
        self.edgeIDList = traci.edge.getIDList()
        for edgeID in self.edgeIDList:
            self.edge_map[edgeID] = Edge(edgeID)
        self.laneIDList = traci.lane.getIDList()
        for laneID in self.laneIDList:
            successor_lane = [t[0] for t in traci.lane.getLinks(laneID)]
            lane_edge = traci.lane.getEdgeID(laneID)
            self.edge_map[lane_edge]._append_lane(laneID)
            successor_edge_list = [traci.lane.getEdgeID(t) for t in successor_lane]
            self.edge_map[lane_edge]._append_successor(successor_edge_list)
            for successor_edge in successor_edge_list:
                self.edge_map[successor_edge]._append_predecessor(lane_edge)

    def get_edge_map(self):
        return self.edge_map


class Edge:
    def __init__(self, edgeID):
        self.edgeID = edgeID
        self.successor_edge = []
        self.lane_list=[]
        self.predecessor_edge = []
        self.crossing_time = 0

    def _append_lane(self, lane=None):
        if lane is not None:
            self.lane_list.append(lane)

    def _append_successor(self, successor=None):
        if successor is not None:
            self.successor_edge.extend(successor)
            self.successor_edge = list(set(self.successor_edge))

    def _append_predecessor(self, predecessor=None):
        if predecessor is not None:
            self.predecessor_edge.append(predecessor)
            self.successor_edge = list(set(self.predecessor_edge))

    def get_lane(self):
        return self.lane_list

    def get_num_lane(self):
        return len(self.lane_list)

    def get_edge_id(self):
        return self.edgeID

    def get_successor(self):
        return self.successor_edge

    def get_predecessor(self):
        return self.predecessor_edge

    # TODO: implement get_crossing_time method
    def get_crossing_time(self):
        pass

    def get_length(self):
        sum_of_length = 0.0
        for lane in self.lane_list:
            sum_of_length += traci.lane.getLength(lane)
        return sum_of_length / self.get_num_lane()

    def get_middle_point(self):
        sum_of_point = 0.0
        for lane in self.lane_list:
            sum_of_point += np.average(traci.lane.getShape(lane), axis=0)
        return sum_of_point / self.get_num_lane()

    def get_car_number(self):
        return traci.edge.getLastStepVehicleNumber(self.edgeID)

# ======================================================================================== #
# if __name__ == "__main__":
#     sumoBinary = checkBinary('sumo-gui')
#     traci.start([sumoBinary, "-c", "../testcase_liverpool_small/osm.sumocfg",
#                  "--tripinfo-output", "tripinfo.xml"])
#     test_graph = SumoGraph()
#     test_map = test_graph.get_edge_map()
#     for edgeID, edge in test_map.items():
#         print(edgeID)
#     print(test_map['11810620'].get_successor())
#     print(test_map['11810620'].get_predecessor())
#     print(test_map['11810620'].get_middle_point())
#     print(test_map['11810620'].get_length())
#     print(test_map['11810620'].get_lane())
#     print(test_map['11810620'].get_num_lane())
#     # execute the TraCI control loop
#     step = 0
#     # we start with phase 2 where EW has green
#     while traci.simulation.getMinExpectedNumber() > 0:
#         traci.simulationStep()
#         step += 1
#     traci.close()
#     sys.stdout.flush()
# ======================================================================================== #

# ======================================================================================== #
# Output:
# [':105729906_3', '11810622#1', ':105729906_0', '-11810627#0']
# ['-11810627#0', '11810622#1', ':105729906_0', ':105729906_3']
# [ 2142.58   1400.905]
# 46.03
# ['11810620_0']
# 1
# ======================================================================================== #

