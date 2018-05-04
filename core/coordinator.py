import collections
import traci
from interface.sumo_interface import Connection
from router.routing import astar, dijkstra
import tensorflow as tf
from copy import deepcopy
from router.estimator import *
import logging
import pandas as pd
from pprint import pprint
import numpy as np
import os, sys
liverpool_dir = "../../sumo/test_cases/liverpool_small_peak/osm.sumocfg"
grid_dir = "../../sumo/test_cases/grid_33/grid_33.sumocfg"

class Coordinator:
    """
    :param map: city map stored as Map class
    :param cars: dictionary of the car_ids to cars
    :param router: Router class which give the car a new route based on Planner
    :param interface: the interface to interact with simulator such as sumo
    :param model path: path to stored model
    :param painter: the painter to draw gif summarising the painting process
    :param result_dir: result directory
    """
    def __init__(self, map, cars, router, interface, estimator, model_path, painter=None, result_dir=None):
        self.step = 0
        self.map = map
        self.cars = cars
        self.__global_router = router
        self.interface = interface
        self._estimator = estimator
        self.update_car_info()
        if model_path:
            self._saver = tf.train.Saver()
        self._model_path = model_path
        self._painter = painter
        self._result_dir = result_dir


    # update routes of all cars
    def update_route(self, verbose=False):
        route = self.__global_router(self.map, self._estimator, self.cars.values())
        for car in self.cars.values():
            self.interface.deploy_route(car.car_id, route[car.car_id])

    def update_car_info(self):
        self.interface.update_car_ids()
        self.cars = self.interface.cars_id

    def finish_step(self):
        pass

    def restore_model(self):
        try:
            self._records = pd.read_csv(self._model_path + ".csv", index_col="epochs")
            self._epochs = self._records.index.max() + 1
            self._saver.restore(self._estimator.session, self._model_path + ".ckpt")
        except Exception as e:
            logging.error(e)

    # main loop
    def simulate(self, verbose=True):
        self.step=0
        while (not self.interface.all_arrival() and self.step < 5000):
            # TODO: control the steps/ ticks of updating
            self.step += 1
            self.update_car_info()
            self.update_route(verbose=verbose)
            self.interface.simulate()
            if self._estimator is not None:
                self._estimator.update(self.cars)
            if self._painter and self.step % 20==0:
                self._painter.append_frame_weights(self._estimator.pheromones)
            self.finish_step()

    def start(self):
        self.restore_model()
        self.simulate()
        self.interface.close()
        if self._painter:
            self._painter.print("simulationResults/"+self._result_dir)

class TravelTimeRecorded(Coordinator):
    def __init__(self, map, cars, router, interface, estimator, model_path):
        Coordinator.__init__(self, map, cars, router, interface, estimator, model_path)

Sample = collections.namedtuple("Sample", "edge pheromone arrival_time travel_time")
Label = collections.namedtuple("Label", "step travel_time")
Input = collections.namedtuple("Input", "step pheromone")
Pair = collections.namedtuple("Pair", "car_id edge_id")

class TrainCoordinator(Coordinator):
    def __init__(self, map, cars, router, interface, estimator, model_path, reset_handle):
        Coordinator.__init__(self, map, cars, router, interface, estimator, model_path)
        self.__lables = {} # dictionary of pair to label
        self.__inputs = {} # dictionary of pair to inputs
        self.__samples = {} # dictionary of edge to list of samples
        self.__car_records = {} # dictionary of carid to steps.
        self.__old_location = {} # dictionary of carid to location
        self.__car_global_steps = {} # dictionary of carid to steps
        self._epochs = 0
        self.__reset = reset_handle
        self._records = pd.DataFrame(columns=["time", "epochs", "average_finish_time", "average_loss"]).set_index("epochs")

    def __init_collection(self):
        del self.__lables
        self.__lables = {} # dictionary of pair to label
        del self.__inputs
        self.__inputs = {} # dictionary of pair to inputs
        del self.__samples
        self.__samples = {} # dictionary of edge to list of samples
        del self.__car_records
        self.__car_records = {} # dictionary of carid to steps.
        del self.__old_location
        self.__old_location = {} # dictionary of carid to location
        del self.__car_global_steps
        self.__car_global_steps = {} # dictionary of carid to steps

    def collect_labels(self):
        for id in self.__car_global_steps:
            self.__car_global_steps[id] += 1
        for id in self.__car_records:
            self.__car_records[id] += 1

        new_cars = self.cars
        for id, car in new_cars.items():
            if id not in self.__car_global_steps:
                self.__car_global_steps[id] = 0
                self.__car_records[id] = 0
            else:
                if id in self.__old_location:
                    change_edge, _ = car.location.difference(self.__old_location[id])
                else:
                    change_edge = False
                if change_edge:
                    travel_time = self.__car_records[id]
                    # arrival step and travel time
                    self.__lables[Pair(id, self.__old_location[id].edge.id)] = Label(self.step-travel_time, travel_time)
                    self.__car_records[id] = 0
        for id in self.cars:
            self.__old_location[id] = self.cars[id].location

    def collect_inputs(self):
        for id, car in self.cars.items():
            if Pair(id, car.location.edge.id) not in self.__inputs:
                self.__inputs[Pair(id, car.location.edge.id)] = Input(self.step, deepcopy(self._estimator.pheromones))

    def finish_step(self):
        self.collect_inputs()
        self.collect_labels()

    def finish_episode(self):
        self.__collect_data()
        pprint(self.__samples)

    def __collect_data(self):
        for pair, label in self.__lables.items():
            if pair not in self.__inputs:
                continue
            if pair.edge_id not in self.__samples:
                self.__samples[pair.edge_id] = []

            for input_pair, inputs in self.__inputs.items():
                if pair.car_id == input_pair.car_id and (label.step - inputs.step)>=0:
                    self.__samples[pair.edge_id].append(Sample(pair.edge_id, inputs.pheromone.get_pheromones(pair.edge_id),
                                                               label.step - inputs.step, label.travel_time))

    def start(self, steps=20):
        self.restore_model()
        first = True
        while self._epochs<50:
            if not first:
                self.__reset()
                self.update_car_info()
            first = False
            self.__init_collection()
            self.simulate(verbose=False)
            self.__collect_data()

            losses = []
            for i in range(steps):
                losses = self.train()
            avg_travel = np.array(list(self.__car_global_steps.values()), dtype=np.float32).mean()
            avg_loss = np.array(losses).mean()
            logging.info("average finish time is {}".format(avg_travel))
            self._records = self._records.append(pd.DataFrame({"time": [pd.Timestamp.now()], "epochs": [self._epochs],
                                                  "average_finish_time": [avg_travel], "average_loss": [avg_loss]})
                                                 .set_index("epochs"))
            self._saver.save(self._estimator.session, self._model_path + ".ckpt")
            self._records.to_csv(self._model_path + ".csv")
            self._epochs+=1

    def train(self, n_epoch=100):
        losses = []
        for edge_id, samples in self.__samples.items():
            if len(samples) > 0:
                input_data = []
                labels = []
                for sample in samples:
                    sample_data = slice_pheromone(sample.pheromone, sample.arrival_time)
                    input_data.append(sample_data)
                    labels.append([sample.travel_time])
                loss = self._estimator.train(np.array(input_data), np.array(labels), edge_id, n_epoch)
                losses.append(loss)
        logging.info("average loss is:" + str(np.array(losses).mean()))
        return losses

if __name__ == "__main__":
    logging.basicConfig(level=logging.DEBUG)
    sumo_interface = Connection(dir=liverpool_dir, use_gui=True)
    sumo_interface.extract_map()
    map = sumo_interface.map

    coordinator = TrainCoordinator(map, None, astar, sumo_interface, NNEstimator(sumo_interface.cars_id, map, train_count_dir="../models/liverpool"),
                                   model_path="../models/liverpool",
                                   reset_handle=lambda: sumo_interface.reset(dir=liverpool_dir))
    coordinator.start()
    sumo_interface.close()
    sys.stdout.flush()
