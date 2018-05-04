from __future__ import division, absolute_import, print_function
from traffic_info.map import *

class Car():
    def __init__(self, current, destination, route, carid):
        '''

        :param current: start location
        :param destination: destination location
        :param route: list of edges
        :param carid: string, id of the car
        '''
        self.__current = current
        self.__destination = destination
        self.__route = route
        self.__step = 0
        self.__id = carid

    @property
    def route(self):
        return self.__route

    @property
    def id(self):
        return self.__id

    @property
    def step(self):
        return self.__step

    @property
    def location(self):
        return self.__current

    @property
    def destination(self):
        return self.__destination

    @property
    def car_id(self):
        return self.__id

    def increment_step(self):
        self.__step += 1

    def update_current(self, current_loc):
        self.__current = current_loc