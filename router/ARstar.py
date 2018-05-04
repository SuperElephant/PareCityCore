from __future__ import division, absolute_import, print_function
import numpy as np
import heapq as heapq
import itertools
from traffic_info.map import Map, Edge, Location
from traffic_info.car import Car


# priority queue for astar
class PriorityQueue:
    REMOVED = '<removed-task>'  # placeholder for a removed task

    def __init__(self):
        self.elements = []
        self.entry_finder = {}
        # counter = itertools.count()  # unique sequence count

    # mark an item as REMOVED
    def remove(self, item):
        entry = self.entry_finder.pop(item)
        entry[-1] = self.REMOVED

    # add a new item or update the priority
    def put(self, item, priority):
        if item in self.entry_finder:
            self.entry_finder.pop(item)
        # count = next(self.counter)
        entry = [priority, item]
        self.entry_finder[item] = entry
        heapq.heappush(self.elements, entry)

    # remove and return the lowest priority item
    def get(self):
        while self.elements:
            priority, item = heapq.heappop(self.elements)
            if item in self.entry_finder:
                del self.entry_finder[item]
                return item
        raise KeyError('pop from an empty priority queue')

    def getitem(self):
        return self.entry_finder


def _calculate_footprint(length, edge, vehicle_number):
    """
    This method is used to calculate footprint of an edge
    """
    avg_length = length
    lane = edge.width
    if lane == 0:
        lane = 1
    if edge.id in vehicle_number:
        n = vehicle_number[edge.id] + 1
    else:
        n = 1
    w = avg_length / (edge.length * lane)
    fc = w * n
    return fc


def _calculate_distance(current, goal):
    """
    This method is used to calculate the spatial distance between two edges
    """
    start = current.position
    end = goal.position
    distance = np.sqrt(np.sum(np.square(start - end)))
    return distance


def _sumFscore(FScore, agenda):
    """
    This method is used to calculate the sum of FScore
    """
    sum_fscore = 0
    entries = agenda.getitem()
    for item in entries:
        sum_fscore = sum_fscore + FScore[item.id]
    return sum_fscore


def _sumRscore(RScore, agenda):
    """
    This method is used to calculate the sum of RScore
    """
    sum_gscore = 0
    entries = agenda.getitem()
    for item in entries:
        sum_gscore = sum_gscore + RScore[item.id]
    return sum_gscore


def _reconstruct_path(came_from, start, goal):
    """
    This method is used to construct the path from start edge to goal edge
    """
    current = goal
    path = []
    while current != start:
        path.append(current)
        current = came_from.get(current)
    path.append(start)
    path.reverse()
    return path


def arstar(graph, estimator, cars):
    '''
        This is the implementation of ARstar algorithm
        :param graph: map
        :param start: start location
        :param destination: destination location
        :param estimator: Estimator, used to generate estimated travel time
        :param cars: list of cars
        :return: path id for each car in cars
    '''

    repulation_weight = 1
    car_routes = {}
    vehicle_number = {}

    for car in cars:
        agenda = PriorityQueue()
        closeset = []
        GScore = {}
        HScore = {}
        FScore = {}
        RScore = {}
        came_from = {}

        start = car.location
        destination = car.destination
        begin = start.edge
        goal = destination.edge

        # GScore[begin.id] = estimator.estimate_time(begin, car, 0)
        GScore[begin.id] = begin.length
        HScore[begin.id] = _calculate_distance(begin, goal)

        FScore[begin.id] = GScore[begin.id] + HScore[begin.id]
        agenda.put(begin, FScore[begin.id])
        came_from[begin.id] = None

        # not sure
        sum_edge_length = 0
        graph_edges = graph.get_edges()
        for edge_id, edge in graph_edges.items():
            sum_edge_length = sum_edge_length + edge.length
        avg_length = sum_edge_length / len(graph_edges)
        RScore[begin.id] = _calculate_footprint(avg_length, begin, vehicle_number)

        while agenda:
            sumF = _sumFscore(FScore, agenda)
            sumR = _sumRscore(RScore, agenda)
            items = agenda.getitem()
            for item in items:
                FScore[item.id] = (1-repulation_weight) * FScore[item.id] / sumF + repulation_weight * RScore[item.id] / sumR
                # update edge with new priority FScore
                agenda.put(graph.get_edge(item.id), FScore[item.id])

            # get the edge with smallest cost,agenda remove current
            current = agenda.get()

            if current == goal:
                break
            else:
                # closeset add current
                closeset.append(current)

            for next_id in current.downflow_ids:
                next = graph.get_edge(next_id)
                if next in closeset:
                    continue

                #tentative_g = GScore[current.id] + estimator.estimate_time(next, car, GScore[current.id])
                tentative_g = GScore[current.id] + next.length
                tentative_r = RScore[current.id] + _calculate_footprint(avg_length, next, vehicle_number)

                items = agenda.getitem()
                if next not in items:
                    came_from[next.id] = current.id
                    # agenda.put(next, priority)
                    HScore[next.id] = _calculate_distance(next, goal)
                    tentative_is_better = True

                else:
                    if tentative_g < GScore[next.id]:
                        tentative_is_better = True
                    else:
                        tentative_is_better = False

                if tentative_is_better:
                    # P[edge]
                    GScore[next.id] = tentative_g
                    RScore[next.id] = tentative_r
                    FScore[next.id] = GScore[next.id] + HScore[next.id]
                    agenda.put(next, FScore[next.id])

        route = _reconstruct_path(came_from, begin.id, goal.id)
        for edge_id in route:
            if edge_id in vehicle_number:
                vehicle_number[edge_id] = vehicle_number[edge_id] + 1
            else:
                vehicle_number[edge_id] = 1
        car_routes[car.id] = route

    return car_routes

# used for testing
if __name__ == "__main__":
    edge_info = [
        ("A", 4, np.array([2, 4]), ["B", "D"]),
        ("B", 4, np.array([6, 4]), ["H", "C"]),
        ("C", 4, np.array([10, 4]), ["I"]),
        ("D", 4, np.array([4, 2]), ["E"]),
        ("E", 4, np.array([6, 0]), ["F"]),
        ("F", 4, np.array([10, 0]), ["G"]),
        ("G", 4, np.array([14, 0]), ["J"]),
        ("H", 4, np.array([8, 2]), ["F"]),
        ("I", 4, np.array([12, 2]), ["G"]),
        ("J", 4, np.array([16, 2]), ["K"]),
        ("K", 4, np.array([14, 4]), ["I"]),
    ]
    edges = []
    for info in edge_info:
        edges.append(Edge(info[0], info[1], 0, info[2], None, info[3]))
    graph = Map.from_edges(edges)

    start = Location(graph.get_edge("A"), 0)
    end = Location(graph.get_edge("G"), 2)
    vehicle1 = Car(start, end, [], "v1")
    vehicle2 = Car(start, end, [], "v2")
    vehicle3 = Car(start, end, [], "v3")

    routes = arstar(graph, None, [vehicle1, vehicle2, vehicle3])
    route_v1 = routes["v1"]
    route_v2 = routes["v2"]
    route_v3 = routes["v3"]

    print("A -> G")
    print("Path of vehicle 1: ")
    print(*[edge for edge in route_v1], sep=",")

    print("Path of vehicle 2: ")
    print(*[edge for edge in route_v2], sep=",")

    print("Path of vehicle 3: ")
    print(*[edge for edge in route_v3], sep=",")




