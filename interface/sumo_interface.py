import os, sys
import re
import numpy as np
import logging
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

from traffic_info.car import *

class Connection:
    def __init__(self, dir, use_gui=False, summary_dir=""):
        self.start(dir, use_gui, summary_dir)
        self.__edge_id_list = traci.edge.getIDList()
        self.__lane_id_list = traci.lane.getIDList()
        self.__downflow_edges = {}
        self.__upperflow_edges = {}
        self.__lanes_of_edges = {}
        self.__map = Map()
        self.__cars = {}
        self.__cars_id = {}
        self.__travel_time_sample = {}
        self.__time_step = 0
    """
    Extract map by using traci 
    :return Map object 
    """
    def extract_map(self):
        for lane_id in self.__lane_id_list:
            if 'passenger' not in traci.lane.getDisallowed(lane_id) and re.match(':', lane_id) is None:
                downflow_lanes = set(t[0] for t in traci.lane.getLinks(lane_id)
                                     if 'passenger' not in traci.lane.getDisallowed(t[0])
                                     and re.match(':', t[0]) is None)
                downflow_edges_of_lane = set(traci.lane.getEdgeID(lane) for lane in downflow_lanes)
                edge_this_lane = traci.lane.getEdgeID(lane_id)
                if self.__lanes_of_edges.get(edge_this_lane) is None:
                    self.__lanes_of_edges[edge_this_lane] = [lane_id]
                else:
                    self.__lanes_of_edges[edge_this_lane].append(lane_id)

                if self.__downflow_edges.get(edge_this_lane) is None:
                    self.__downflow_edges[edge_this_lane] = downflow_edges_of_lane
                else:
                    self.__downflow_edges[edge_this_lane] = self.__downflow_edges[edge_this_lane] | downflow_edges_of_lane

                for downflow_edge_of_lane in downflow_edges_of_lane:
                    if self.__upperflow_edges.get(downflow_edge_of_lane) is None:
                        self.__upperflow_edges[downflow_edge_of_lane] = {edge_this_lane}
                    else:
                        self.__upperflow_edges[downflow_edge_of_lane].add(edge_this_lane)

        for edge_id in self.__edge_id_list:
            if self.__lanes_of_edges.get(edge_id) is not None:
                width = len(self.__lanes_of_edges[edge_id])

                sum_of_length = 0.0
                for lane in self.__lanes_of_edges[edge_id]:
                    sum_of_length += traci.lane.getLength(lane)
                edge_length = sum_of_length / width

                sum_of_point = 0.0
                for lane in self.__lanes_of_edges[edge_id]:
                    sum_of_point += np.average(traci.lane.getShape(lane), axis=0)
                middle_point = sum_of_point / width

                if self.__upperflow_edges.get(edge_id) is None:
                    self.__upperflow_edges[edge_id] = set()

                if self.__downflow_edges.get(edge_id) is None:
                    self.__downflow_edges[edge_id] = set()

                self.__map.add_edge(edge_id, Edge(edge_id, edge_length, width, middle_point,
                                                  self.__upperflow_edges[edge_id], self.__downflow_edges[edge_id]))


    def deploy_route(self, car_id, edges):
        traci.vehicle.setRoute(car_id, edges)

    def get_car_current_loc(self, car_id,):
        return traci.vehicle.getRoadID(car_id)

    # Update Car object each time step
    def update_car_ids(self):
        del self.__cars_id
        self.__cars_id = {}
        for veh_id in traci.simulation.getDepartedIDList():
            traci.vehicle.subscribe(veh_id, [traci.constants.VAR_ROAD_ID, traci.constants.VAR_EDGES, traci.constants.VAR_LANEPOSITION])
        for car_id in traci.vehicle.getIDList():
            result = traci.vehicle.getSubscriptionResults(car_id)
            current = result[traci.constants.VAR_ROAD_ID]
            if current == "":
                print("%s has been removed from simulation because of collision", car_id)
            elif re.match(':', current) is None:
                edges = result[traci.constants.VAR_EDGES]
                distance = result[traci.constants.VAR_LANEPOSITION]
                current_loc = Location(self.__map.get_edge(current), distance)
                destination = edges[-1]
                dest_distance = self.__map.get_edge(destination).length
                dest_loc = Location(self.__map.get_edge(destination), dest_distance)
                self.__cars_id[car_id] = Car(current_loc, dest_loc, edges, car_id)

    def start(self, dir, use_gui=False, summary_dir=""):
        if use_gui:
            sumoBinary = checkBinary('sumo-gui')
        else:
            sumoBinary = checkBinary('sumo')

        if not os.path.exists("simulationResults/"):
            os.makedirs("simulationResults/")
        if summary_dir:
            traci.start([sumoBinary, "-c", dir, "--summary", "simulationResults/"+summary_dir+".xml"])
        else:
            traci.start([sumoBinary, "-c", dir, "--summary", "simulationResults/temp.xml"])

    def reset(self, dir, use_gui=False):
        traci.close(False)
        sys.stdout.flush()
        if use_gui:
            sumoBinary = checkBinary('sumo-gui')
        else:
            sumoBinary = checkBinary('sumo')
        traci.start([sumoBinary, "-c", dir])

    def close(self):
        traci.close(False)
        sys.stdout.flush()

    def all_arrival(self):
        return traci.simulation.getMinExpectedNumber() <= 0

    def simulate(self):
        traci.simulationStep()
        self.add_time_step()

    def add_time_step(self):
        self.__time_step += 1

    @property
    def map(self):
        return self.__map

    @property
    def travel_time_sample(self):
        return self.__travel_time_sample

    @property
    def cars(self):
        return self.__cars

    @property
    def cars_id(self):
        return self.__cars_id

# =======================================================test=======================================================#
if __name__ == "__main__":
    sumoBinary = checkBinary('sumo')
    traci.start([sumoBinary, "-c", "../../sumo/test_cases/grid_33/grid_33.sumocfg",
                 "--tripinfo-output", "tripinfo.xml"])

    print(traci.edge.getIDList())
    print(traci.vehicle.getIDList())

    connection = Connection()
    connection.extract_map()
    map = connection.map
    print(map.map)
    # execute the TraCI control loop
    step = 0
    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()
        step += 1
        connection.initialize_car()
        connection.update_travel_time()
        connection.update_car_ids()
        # for car_id, value in connection.cars_id.items():
        #     print(car_id, value.location)
        connection.add_time_step()

        if step == 100:
            print(traci.vehicle.getIDList())
            print(traci.vehicle.getRoute('vid_0'))
            print(traci.vehicle.getRoadID('vid_0'))
            current_edge = traci.vehicle.getRoadID('vid_0')
            connection.deploy_route('vid_0', [current_edge, 'L07', 'L08'])


        if step == 150:
            print(traci.vehicle.getRoadID('vid_0'))
            print(traci.vehicle.getRoute('vid_0'))
            print(connection.cars)
            print(connection.travel_time_sample)
    print(connection.cars)
    print(connection.travel_time_sample)
    traci.close()
    sys.stdout.flush()
# =======================================================test=======================================================#
