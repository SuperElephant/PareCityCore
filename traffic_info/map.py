from __future__ import division, absolute_import, print_function

class Map:
    def __init__(self):
        self.__edges = {}

    @staticmethod
    def from_edges(edges):
        graph = Map()
        for edge in edges:
            graph.__edges[edge.id] = edge
        return graph

    def add_edge(self, id, edge):
        self.__edges[id] = edge

    def get_edge(self, id):
        return self.__edges[id]

    def get_upperflow(self, edge):
        return [self.__edges[upper_id] for upper_id in edge.upperflow_ids]

    def get_downflow(self, edge):
        return [self.__edges[down_id] for down_id in edge.downflow_ids]

    # use in ARstar
    def get_edges(self):
        return self.__edges

    def __iter__(self):
        return self.__edges.values().__iter__()

    # use for test
    @property
    def map(self):
        return self.__edges

class Edge:
    def __init__(self, id, length, width, position, upperflow, downflow):
        """
        creat an unconnected edge
        :param id: id of edge
        :param length: length of the road
        :param width: numberr of lanes
        :param position: numpy array size (,2), start point of the edge
        :param upperflow: set of ids of upperflow edges
        :param downflow: set of ids of downflow edges
        """
        self.__id = id
        self.__length = length
        self.__width = width
        self.__position = position
        self.__upperflow = upperflow
        self.__downflow = downflow

    @property
    def id(self):
        return self.__id

    @property
    def upperflow_ids(self):
        return self.__upperflow

    @property
    def downflow_ids(self):
        return self.__downflow

    @property
    def length(self):
        return self.__length
    @property
    def position(self):
        return self.__position

    @property
    def width(self):
        return self.__width

    @property
    def position(self):
        return self.__position

    def __hash__(self):
        return hash(self.__id)

    def __lt__(self, other):
        return self.length < other.length

    def __eq__(self, other):
        """Overrides the default implementation"""
        if isinstance(self, other.__class__):
            return self.id == other.id
        return False

    def __ne__(self, other):
        """Overrides the default implementation (unnecessary in Python 3)"""
        return not self.__eq__(other)

class Location:
    """
    Edge based location
    """
    def __init__(self, edge, distance):
        self.__edge = edge
        self.__distance = distance

    @property
    def edge(self):
        return self.__edge

    @property
    def distance(self):
        return self.__distance

    @property
    def remain(self):
        return self.__edge.length-self.__distance

    def difference(self, target):
        '''
        :param target: Location object, target location, must be a adjacent edge
        :return: tuplue of (boolean, float), if the vehicle switch the edge and the distance between edges
        '''
        if self.__edge.id == target.edge.id:
            return False, abs(self.__distance - target.distance)
        if target.edge.id in self.__edge.downflow_ids:
            return True, target.distance+self.remain
        elif target.edge.id in self.__edge.upperflow_ids:
            return True, target.remain+self.distance
        else:
            return True, 0
            #raise ValueError("the two edges are not adjacent")


