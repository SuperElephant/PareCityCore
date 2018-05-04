from __future__ import division, absolute_import, print_function
from collections import defaultdict
from heapq import *
import numpy as np
import heapq as heapq
from traffic_info.map import Map, Edge, Location

def dijkstra(source, destination, graph, estimator):
    """
    This method is an implementation of Dijkstra algorithm
    :param source: Edge, start edge
    :param destination: Edge, destination edge
    :param graph: Map
    :param estimator: Estimator, used to generate estimated travel time
    :return:
    """
    heap = [(source.length, source, ())]  # cost, edge, path
    seen = set() # set of edges
    while heap:
        (cost, current, path) = heappop(heap)
        if current not in seen:
            seen.add(current)
            path = (current,)+ path
            if current == destination: return (cost, reversed(path))

            for downflow in graph.get_downflow(current):
                if downflow not in seen:
                    # heappush(heap, (cost + estimator.estimate_time(downflow, car, cost), downflow, path))
                    heappush(heap, ((cost + downflow.length), downflow, path))

    return float("inf")


# priority queue for Astar
class PriorityQueue:
    def __init__(self):
        self.elements = []

    def empty(self):
        return len(self.elements) == 0

    def put(self, item, priority):
        heapq.heappush(self.elements, (priority, item))

    def get(self):
        return heapq.heappop(self.elements)[1]

def global_astar(graph, esimator, cars):
    '''
    unify apis to ARstar
    :param graph:
    :param esimator:
    :param cars:
    :return:
    '''
    results = {}
    for car in cars:
        results[car.id] = astar(car.location, car.destination, graph, esimator)
    return results

def astar(start, destination, graph, estimator):
    '''
    This method is an implementation of Astar rerouting algorithm
    :param graph: map
    :param start: start location
    :param destination: destination location
    :param estimator: Estimator, used to generate estimated travel time
    :return: path id
    '''
    begin = start.edge
    goal = destination.edge
    agenda = PriorityQueue()
    agenda.put(begin, 0 + _calculate_distance(begin, goal))

    '''
    :came_from: record the connection between the passing edges, key and value is edge id
    :arrivial_time: the overal travel time when passing the edge n, key is edge id
    :route: return value
    '''
    came_from = {}
    arrivial_time = {}
    route = []

    came_from[begin.id] = None
    arrivial_time[begin.id] = (1-start.distance/begin.length) * estimator.estimate_time(begin, 0)

    while agenda:
        # get the edge with smallest cost
        current = agenda.get()
        #current = graph.get_edge(current_id)
        if current == goal:
            break
        for next_id in current.downflow_ids:
            next = graph.get_edge(next_id)
            gn = arrivial_time[current.id] + estimator.estimate_time(next, arrivial_time[current.id])
            #gn = arrivial_time[current.id] + next.length
            if next_id not in arrivial_time or gn < arrivial_time[next_id]:
                arrivial_time[next_id] = gn
                hn = _calculate_distance(next, goal)
                priority = gn + hn
                agenda.put(next, priority)
                came_from[next_id] = current.id

    route = _reconstruct_path(came_from, begin.id, goal.id)
    return route


def _calculate_distance(current, goal):
    """
    This method is used to calculate the spatial distance between two edges
    """
    start = current.position
    end = goal.position
    distance = np.sqrt(np.sum(np.square(start - end)))
    return distance


def _reconstruct_path(came_from, start, goal):
    """
        This method is used to construct path from start to goal
    """
    current = goal
    path = []
    while current != start:
        path.append(current)
        current = came_from.get(current)
    path.append(start)
    path.reverse()
    return path


# used for test
if __name__ == "__main__":
    edge_info = [
        ("A", 7, ["B", "D"]),
        ("B", 5, ["C"]),
        ("C", 8, ["F"]),
        ("D", 1, ["E"]),
        ("E", 7, ["F"]),
        ("F", 5, ["A"]),
    ]
    edges = []
    for info in edge_info:
        edges.append(Edge(info[0], info[1], 0, None, None, info[2]))
    graph = Map.from_edges(edges)

    print("=== Dijkstra ===")
    print(edge_info)
    print("A -> F:")
    distance, path = dijkstra(graph.get_edge("A"), graph.get_edge("F"), graph, None)
    print("distance is:" + str(distance))
    print("path is:")
    print(*[edge.id for edge in path], sep=",")
    print("E -> A:")
    distance, path = dijkstra(graph.get_edge("E"), graph.get_edge("A"), graph, None)
    print("distance is:" + str(distance))
    print("path is:")
    print(*[edge.id for edge in path], sep=",")

    edge_info = [
        ("A", 8, np.array([4, 0]), ["B", "D"]),
        ("B", 6, np.array([8, 3]), ["C"]),
        ("C", 8, np.array([4, 6]), ["F"]),
        ("D", 2, np.array([8, -1]), ["E"]),
        ("E", 8, np.array([4, -2]), ["G"]),
        ("F", 6, np.array([0, 3]), ["A"]),
        ("G", 2, np.array([0, -1]), ["A"]),
    ]
    edges = []
    for info in edge_info:
        edges.append(Edge(info[0], info[1], 0, info[2], None, info[3]))
    graph = Map.from_edges(edges)

    print("=== Astar ===")
    print(edge_info)
    print("A -> F:")
    start = Location(graph.get_edge("A"), 0)
    end = Location(graph.get_edge("F"), 0)
    distance, path = astar(start, end, graph, None)
    print("distance is:" + str(distance))
    print("path is:")
    print(*[edge for edge in path], sep=",")

    print("E -> A:")
    start = Location(graph.get_edge("E"), 0)
    end = Location(graph.get_edge("A"), 0)
    distance, path = astar(start, end, graph, None)
    print("distance is:" + str(distance))
    print("path is:")
    print(*[edge for edge in path], sep=",")
