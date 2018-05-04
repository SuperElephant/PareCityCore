from argparse import ArgumentParser
from interface.sumo_interface import Connection
from core.coordinator import Coordinator, TrainCoordinator
from router.ARstar import *
from router.estimator import *
from router.routing import *
from visualization.painter import GifPainter
import os
import logging
import pandas as pd
import matplotlib.pyplot as plt

liverpool_dir = "../sumo/test_cases/liverpool_small_peak/osm.sumocfg"
grid_dir = "../sumo/test_cases/grid_33/grid_33.sumocfg"

def build_parser():
    parser = ArgumentParser()
    parser.add_argument("--mode",dest="mode",
                        help="can be train or simulate",
                        default="train")
    parser.add_argument("--map", dest="map",
                        help="the map simulation will run on, can be grid, gridCross, gridSide2Corner, gridSingle, liverpool",
                        default="grid")
    parser.add_argument("--gui", dest="gui", type=bool,
                        help="adding this parameter, the sumo gui will be turned on",
                        default=False)
    parser.add_argument("--logging", dest="log_level",
                        help="log level, can be debug, info, warning or error",
                        default="info")
    parser.add_argument("--router", dest="router",
                        help="routing algorithm, can be astar, arstar, or nnrouter",
                        default="nnrouter")
    parser.add_argument("--model", dest="model",
                        help="name of the model. The program will try to restore the model if there is"
                             "such one in the models folder. Trained results will also be saved with this"
                             "name. Required only if the router is nnrouter.")
    parser.add_argument("--result", dest="result_dir",
                        help="directory of simulation result to be output.",
                        default="")
    return parser

def get_map_dir(name):
    if name=="grid":
        return grid_dir
    elif name=="liverpool":
        return liverpool_dir
    elif name=="gridCross":
        return "../sumo/test_cases/grid_33_cross/grid_33.sumocfg"
    elif name=="gridSide2Corner":
        return "../sumo/test_cases/grid_33_side2corner/grid_33.sumocfg"
    elif name=="gridSingle":
        return "../sumo/test_cases/grid_33_single_jam/grid_33.sumocfg"

def get_net_file(name):
    return get_map_dir(name).replace(".sumocfg",".net.xml")

if __name__ == "__main__":
    parser = build_parser()
    options = parser.parse_args()
    if not os.path.exists("./" + "models"):
        os.makedirs("./" + "models")
    dir = get_map_dir(options.map)
    logging.basicConfig(level=logging.DEBUG)
    sumo_interface = Connection(dir, options.gui, options.result_dir)
    sumo_interface.extract_map()
    map = sumo_interface.map

    if options.mode=="train":
        coordinator = TrainCoordinator(map, None, global_astar, sumo_interface, NNEstimator(sumo_interface.cars_id, map,
                                       train_count_dir="./models/"+options.model),
                                       model_path="./models/"+options.model,
                                       reset_handle=lambda: sumo_interface.reset(dir, options.gui))
        coordinator.start()
    if options.mode=="simulate":
        if options.router == "astar":
            router = global_astar
            estimator = FixedSpeedEstimator(sumo_interface.cars_id, map)
            model_path = ""
        elif options.router == "arstar":
            router = arstar
            estimator = FixedSpeedEstimator(sumo_interface.cars_id, map)
            model_path = ""
        elif options.router == "nnrouter":
            router = global_astar
            estimator = NNEstimator(sumo_interface.cars_id, map, train_count_dir="./models/"+options.model)
            model_path = "./models/"+options.model
        else:
            raise ValueError("invalid router name")

        coordinator = Coordinator(map, None, router, sumo_interface, estimator, model_path,
                                  GifPainter(net_file=get_net_file(options.map), duration=0.3), options.result_dir)
        coordinator.start()
    if options.mode=="plot":
        data = pd.read_csv("models/"+options.model+".csv")
        ax = data["average_loss"].plot(title=options.model+" training", figsize=(5,3))
        ax.set(xlabel="epochs", ylabel="training loss of new samples")
        plt.tight_layout()
        plt.show()
    sumo_interface.close()
    sys.stdout.flush()


