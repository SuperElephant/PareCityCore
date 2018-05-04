import os
import sys
import numpy as np
from abc import abstractmethod
import tensorflow as tf
import pickle
import logging
from contextlib import contextmanager
import time

@contextmanager
def log_time(prefix=""):
    start = time.time()
    try:
        yield
    finally:
        end = time.time()
        elapsed_seconds = float("%.2f" % (end - start))
        logging.debug('%s: elapsed seconds: %s', prefix, elapsed_seconds)

def slice_pheromone(pheromone, arrival_time):
    arrival_index = int(arrival_time // NNEstimator.time_unit) if arrival_time>0 else 0
    arrival_index = Estimator.estimate_time_period - 1 if arrival_index >= Estimator.estimate_time_period else arrival_index
    padded_pheromone = np.pad(np.array(pheromone), (NNEstimator.slice_shift, NNEstimator.slice_shift), "constant", constant_values=(0, 0))
    intention = padded_pheromone[arrival_index:arrival_index + NNEstimator.slice_shift * 2 + 1]
    input_data = np.concatenate([[pheromone[0]], intention])
    return input_data

class Estimator(object):

    # TODO: better in Coordinator
    """Class used to estimate the travel time """
    estimate_time_period = 100

    def __init__(self, cars, traffic_map, default_average_speed=30):
        super(Estimator, self).__init__()
        self._cars = cars
        self._map = traffic_map
        self.default_average_speed = default_average_speed

    def update(self, cars):
        """
        called by the coordinator
        :return:
        """
        pass

    @abstractmethod
    def estimate_time(self, edge, arrival_time):
        # return the estimate time
        pass


class FixedSpeedEstimator(Estimator):
    # use the average speed
    def __init__(self, cars, traffic_map,default_average_speed=30):
        super(FixedSpeedEstimator, self).__init__(cars, traffic_map,
                                                  default_average_speed=default_average_speed)
        self.__all_pheromones = AllPheromones(traffic_map, self.estimate_time_period)

    def estimate_time(self, edge, arrival_time):
        return edge.length

    def update(self, cars):
        self.__all_pheromones.update_pheromone(self)
        self._cars = cars

    # use for test
    @property
    def pheromones(self):
        return self.__all_pheromones


class NNEstimator(Estimator):
    time_unit = 5
    slice_shift = 3

    def __init__(self, cars, traffic_map, train_count_dir="liverpool", default_average_speed=40):
        super(NNEstimator, self).__init__(cars, traffic_map,
                                                 default_average_speed=default_average_speed)
        self.models = {}
        self.train_ops = {}
        self.count_train = {}
        self.losses = {}
        self.__all_pheromones = AllPheromones(traffic_map, self.estimate_time_period)
        self.__input = tf.placeholder(shape=[None, self.slice_shift*2+2], dtype=tf.float32)
        self.__label = tf.placeholder(shape=[None, 1], dtype=tf.float32)
        self.session = tf.Session()
        self.__train_count_dir = train_count_dir
        self.initialize_all_estimator()
        # init pheromones

    def estimate_time(self, edge, arrival_time):
        if edge.id not in self.count_train or self.count_train[edge.id] < 1000:
            time = edge.length/self.default_average_speed
            return time
        # pheromones - > time
        pheromone = self.__all_pheromones.get_pheromones(edge.id)
        model = self.models[edge.id]
        input_data = slice_pheromone(pheromone, arrival_time)
        input_data = input_data.reshape(1, -1)
        time = self.session.run(model, feed_dict={self.__input: input_data})
        time = time if time>=0 else 0
        return time

    def initialize_all_estimator(self, size=32):
        with log_time("create graph"):
            for edge in self._map:
                with tf.variable_scope("edge_"+edge.id.replace("#", "_")):
                    with tf.device("/device:CPU:0"):
                        network = tf.layers.dense(self.__input, size, activation=tf.nn.relu,
                                                  kernel_initializer=tf.random_normal_initializer(),
                                                  bias_initializer=tf.random_normal_initializer(),
                                                  reuse=False)
                        network = tf.layers.dense(network, 1,
                                                  kernel_initializer=tf.random_normal_initializer(),
                                                  bias_initializer=tf.constant_initializer(edge.length / self.default_average_speed),
                                                  reuse=False)
                        self.models[edge.id] = network
                        loss = tf.losses.mean_squared_error(self.__label, network)
                        self.losses[edge.id] = loss
                        starter_learning_rate = 0.05
                        global_step = tf.get_variable('global_step', shape=[], dtype=tf.int64,
                             initializer=tf.zeros_initializer,
                             trainable=False,)
                        learning_rate = tf.train.exponential_decay(learning_rate=starter_learning_rate,
                                                                   global_step=global_step,
                                                                   decay_steps=5, decay_rate=0.96, staircase=True)
                        optimizer = tf.train.AdamOptimizer(learning_rate=0.01)
                        train_op = optimizer.minimize(loss=loss, global_step=global_step)
                        # print(global_step)
                        self.train_ops[edge.id] = train_op

        with log_time("variables initialization"):
            self.session.run(tf.global_variables_initializer())

        try:
            with open(self.__train_count_dir+".pkl", 'rb') as f:
                self.count_train = pickle.load(f)
        except Exception as err:
            print(err)
            for edge in self._map:
                self.count_train[edge.id] = 0

    def update(self, cars):
        self.__update_pheromone()
        self._cars = cars

    def __update_pheromone(self):
        self.__all_pheromones.update_pheromone(self)

    def train(self, data_input, label, edge_id):
        model = self.models[edge_id]
        # print(edge_id, 'Number Of Samples:', len(input))
        # print(samples)
        loss, _  =self.session.run([self.losses[edge_id] , self.train_ops[edge_id]], feed_dict={self.__label: label, self.__input:data_input})
        self.count_train[edge_id] += len(data_input)
        with open(self.__train_count_dir+".pkl", 'wb') as f:
            pickle.dump(self.count_train, f)
        #model.save('models/' + edge_id + '.h5')
        return loss

    # use for test
    @property
    def pheromones(self):
        return self.__all_pheromones

'''
dependent on map.map(self)
'''
# TODO: or make map iterable
class AllPheromones:
    ''' maintain pheromones'''
    def __init__(self, traffic_map, time_period):
        self.__all_pheromones = {}
        self.__set_zero(traffic_map,time_period)

    def __set_zero(self, traffic_map, time_period):
        for edge in traffic_map:
            self.__all_pheromones[edge.id] = np.zeros(time_period)

    def get_pheromone_at(self, edge_id, time_index):
        return self.get_pheromones(edge_id)[time_index]

    def get_pheromones(self, edge_id):
        return self.__all_pheromones[edge_id]

    def set_pheromone(self, edge_id, new_pheromone):
        self.__all_pheromones[edge_id] = new_pheromone

    def append_pheromone(self, edge_id, time_index, new_pheromone):
        if time_index<len(self.__all_pheromones[edge_id]):
            self.__all_pheromones[edge_id][time_index] += new_pheromone
        else:
            self.__all_pheromones[edge_id][len(self.__all_pheromones[edge_id]-1)] += new_pheromone

    def update_pheromone(self, estimator):
        self.__set_zero(estimator._map, estimator.estimate_time_period)
        for car_id,car in estimator._cars.items():
            route = car.route
            current = car.location.edge.id

            # # TEST: code
            # route = traci.vehicle.getRoute(car_id)
            # current = traci.vehicle.getRoadID(car_id)
            # print('old',route)
            for i in range(len(route)):
                if current == route[i]:
                    route = route[i:]
                    break

            arrival_time = 0
            for edge_id in route:

                if isinstance(estimator,NNEstimator):
                    time_index = int(arrival_time//estimator.time_unit)
                else:
                    time_index = int(arrival_time//3)
                #print(arrival_time, time_index)

                if time_index >= estimator.estimate_time_period:
                    break
                self.append_pheromone(edge_id,time_index,1)
                arrival_time += estimator.estimate_time(estimator._map.get_edge(edge_id), arrival_time)

    def __iter__(self):
        return self.__all_pheromones.items().__iter__()

# test case ==================================================================
if __name__ == '__main__':
    sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
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

    from interface.sumo_interface import *

    """
    This is a test main
    """
    sumoBinary = checkBinary('sumo')


    sumo_interface = Connection(dir="../../sumo/test_cases/grid_33/grid_33.sumocfg")
    # execute the TraCI control loop
    step = 0
    sumo_interface.extract_map()
    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()
        step += 1
        # sumo_interface.initialize_car()
        # sumo_interface.update_travel_time()
        sumo_interface.update_car_ids()
        sumo_interface.add_time_step()
        traffic_map = sumo_interface.map
        cars = sumo_interface.cars_id
        travel_time = sumo_interface.travel_time_sample
        if step == 100:
            """
            TODO: test code here
            active at 100ms
            """

            # test code start============================================
            # TODO: pheromones and add update


            # speed_estimator = TimeSpeedEstimator(cars, traffic_map)
            # print('==TimeSpeedEstimator==')
            # print(speed_estimator.estimate_time(traffic_map.get_edge('-L03'), cars['vid_0'], 0))

        print('==',step,'==')
        for id,car in cars.items():
            print(id, car.route, car.location.edge.id)
        # nn_estimator = NNEstimator(cars,traffic_map)
        # nn_estimator.update(cars)
        # pheromones = nn_estimator.pheromones

        estimator = FixedSpeedEstimator(cars, traffic_map)
        estimator.update(cars)
        pheromones = estimator.pheromones

        for edge in traffic_map:
            if not edge.id.startswith(':'):
                print(edge.id,pheromones.get_pheromones(edge.id))

    traci.close()
    sys.stdout.flush()