
# this file is base on sumo/tools/visualization/plot_net_dump.py
# https://github.com/DLR-TS/sumo/blob/7de0683501f247665bbea78645c2775249cd7dbd/tools/visualization/plot_net_dump.py

from __future__ import absolute_import
from __future__ import print_function
import os
import sys
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '2'
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

import sumolib  # noqa
from sumolib.visualization import helpers
from matplotlib.collections import LineCollection
from matplotlib.backends.backend_agg import FigureCanvasAgg as FigureCanvas

import matplotlib

import traffic_info.map
import traffic_info.car

import numpy
import imageio
from pylab import *
from router.estimator import NNEstimator,FixedSpeedEstimator,Estimator


class WeightsGetter:
    '''Maintain a dict id->weight'''
    __default_weight = 0

    def __init__(self, id2weight = None):
        self.__id2weight = id2weight
        self.__max = 1
        self.__min = 0
        if id2weight==None:
            id2weight={}
        else:
            for id, weight in id2weight.items():
                if weight > self.__max:
                    self.__max = weight
                elif weight < self.__min:
                    self.__min = weight

    def append(self, id, weight):
        self.__id2weight[id] = weight

    def get_weight_of(self, id):
        if self.__id2weight is None or id not in self.__id2weight:
            return self.__default_weight
        return self.__id2weight[id]

    @property
    def max(self):
        return self.__max

    @property
    def min(self):
        return self.__min

class PheromonesGetter (WeightsGetter):
    ''' Maintain a dict id->weight, that accept pheromones directly'''

    def __init__(self, pheromones, time_index):
        self.__time_index = time_index
        current = {}
        for id,p in pheromones:
            current[id] = p[time_index]
        super(PheromonesGetter,self).__init__(current)

    @property
    def time_index(self):
        return self.__time_index


class Painter:
    ''' Draw the graph of weighted map'''

    def __init__(self, weight_getter=None, cmap='cool',time_step="", scale = None,
                 net_filename = '../../sumo/test_cases/grid_33/grid_33.net.xml'):
        self.__weight_getter = weight_getter
        # print(os.getcwd())
        self.__net = sumolib.net.readNet(net_filename)
        self.__colormap = matplotlib.cm.get_cmap(cmap)

        if (scale==None):
            scale = [self.__weight_getter.min, self.__weight_getter.max]

        # set normalize range
        self.__norm = matplotlib.colors.Normalize(vmin=scale[0],
                                                  vmax=scale[1])
        # name set
        time_index = ""
        if isinstance(self.__weight_getter,PheromonesGetter):
            time_index = "_" + str(self.__weight_getter.time_index)
        self.__image_name = "Step"+str(time_step)+time_index
        if(os.path.exists(self.__image_name+".png")):
            nums = 1
            while (os.path.exists(self.__image_name +"("+str(nums)+").png")):
                nums=nums+1
            self.__image_name = self.__image_name +"("+str(nums)+")"


    def print_map(self):
        """print one frame of map"""

        fig = figure()
        # canvas = FigureCanvas(fig)
        ax=fig.add_subplot(111)

        edges_shapes = []
        colors = self.get_color_list()
        w = []
        for e in self.__net._edges:
            edges_shapes.append(e.getShape())
            w.append(1.5)

        # draw roads
        line_segments = LineCollection(edges_shapes, linewidths=w, colors=colors)
        ax = plt.gca()
        ax.add_collection(line_segments)
        ax.set_xmargin(0.1)
        ax.set_ymargin(0.1)
        ax.autoscale_view(True, True, True)

        # set scalar map
        sm = plt.cm.ScalarMappable(cmap=self.__colormap, norm=self.__norm)
        sm._A = []
        plt.colorbar(sm)

        # fig setting
        ax.set_aspect('equal',None,'C')

        # legend()
        # savefig(self.__image_name + ".png")
        fig.savefig("temp.png")

        # plt.show()
        # fig.clf()
        # close()
        # gc.collect()
        # helpers.closeFigure(fig, ax, options, False, expandedOutputNames)
        #fig.show()
        # canvas.draw()
        img = fig2data(fig)
        # img1 = imageio.imread(self.__image_name+".png")

        return img

    def get_color_list(self):
        color_list = []
        for e in self.__net._edges:
            weight = self.__weight_getter.get_weight_of(str(e._id))
            color_list.append(self.__colormap(self.__norm(weight)))
        return color_list

    @property
    def img_name(self):
        return self.__image_name

def fig2data ( fig ):
    """
    :brief Convert a Matplotlib figure to a 4D numpy array with RGBA channels and return it
    :param fig a matplotlib figure
    :return a numpy 3D array of RGBA values
    """
    # draw the renderer
    fig.canvas.draw ( )

    # Get the RGBA buffer from the figure
    w,h = fig.canvas.get_width_height()
    buf = numpy.fromstring ( fig.canvas.tostring_argb(), dtype=numpy.uint8 )
    buf.shape = ( h, w,4 )

    # canvas.tostring_argb give pixmap in ARGB mode. Roll the ALPHA channel to have it in RGBA mode
    buf = numpy.roll ( buf, 3, axis = 2 )
    return buf

class GifPainter:
    ''' make gif'''
    TYPE_REALITY = 0
    TYPE_PREDICTION = 1
    def __init__(self, type = TYPE_REALITY, max_frames = 100, stride=1,
                 net_file="../../sumo/test_cases/grid_33/grid_33.net.xml", duration=1):
        self.duration = duration
        self.net_file = net_file
        self.max_frames = max_frames
        self. stride = stride
        self.type = type
        self.scale=[0,1]
        self.all_weights = []
        self.images=[]

    def set_name(self, name):
        self.name = name

    def append_frame_weights(self, pheromones):
        # append a frame of pheromones
        if self.type == self.TYPE_REALITY:
            # call every step
            self.all_weights.append(PheromonesGetter(pheromones,0))
            self.update_scale(-1)

    def append_all_weights(self, pheromones):
        # append all pheromones at one step
        if self.type == self.TYPE_PREDICTION:
            for i in range(min(self.max_frames,Estimator.estimate_time_period)):
                self.all_weights.append(PheromonesGetter(pheromones,i))
                self.update_scale(i)

    def print(self, name="visualization"):
        for i in range(min(len(self.all_weights), self.max_frames)):
            painter = Painter( self.all_weights[i],
                              net_filename = self.net_file, scale = self.scale)
            self.images.append(painter.print_map())
        imageio.mimsave(name + '.gif', self.images, duration =self.duration)
        if os.path.exists("temp.png"):
            os.remove("temp.png")

    def update_scale(self, index):
        self.scale[0] = min(int(self.all_weights[index].min), self.scale[0])
        self.scale[1] = max(int(self.all_weights[index].max), self.scale[1])


# def generate_gif (img_names):
#     images = []
#     img_names = sorted(img_names)
#     for img_name in img_names:
#         images.append(imageio.imread(img_name+".png"))
#     imageio.mimsave(img_names[0] + '.gif', images, duration =1)
#
# def generate_giff (name, images):
#     imageio.mimsave(name + '.gif', images, duration =1)


# def get_pheromone_timeIndex_fig (pheromones, net_file="../../sumo/test_cases/grid_33/grid_33.net.xml"
#                                  , max_time_period = 100, step=0):
#     img_names = []
#     pheromones_getters = []
#     scale = [0,1]
#     for i in range(min(max_time_period,Estimator.estimate_time_period)):
#         pheromones_getters.append(PheromonesGetter(pheromones,i))
#         scale[0] = min(int(pheromones_getters[i].min), scale[0])
#         scale[1] = max(int(pheromones_getters[i].max), scale[1])
#
#     for i in range(min(max_time_period,Estimator.estimate_time_period)):
#
#         painter = Painter(pheromones_getters[i], time_step=str(step),
#                           net_filename = net_file, scale = scale)
#         painter.print_map()
#         img_names.append(painter.img_name)
#     generate_gif(img_names)
#
# def get_step_pheromone_fig (steps_pheromones,net_file="../../sumo/test_cases/grid_33/grid_33.net.xml"
#                             , start_step=0,stride=1):
#     # steps_pheromones[][]={pheromones_step1,pheromones_step2...}
#     img_names = []
#     pheromone_getters = []
#     scale = [0,1]
#     for i in range(len(steps_pheromones)):
#         step_pheromone = {}
#         for id,p in steps_pheromones[i]:
#             step_pheromone[id] = p[0]
#
#         pheromone_getters.append(WeightsGetter(step_pheromone))
#
#         scale[0] = min(int(pheromone_getters[i].min), scale[0])
#         scale[1] = max(int(pheromone_getters[i].max), scale[1])
#
#     for i in range(len(steps_pheromones)):
#
#         painter = Painter(pheromone_getters[i], time_step=str(start_step+i*stride),
#                           net_filename = net_file, scale = scale)
#         painter.print_map()
#         img_names.append(painter.img_name)
#     generate_gif(img_names)



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

    # without step log
    # traci.start([sumoBinary, "-c", "../../sumo/test_cases/liverpool_small_peak/osm.sumocfg", "--no-step-log", "true"])

    # with step log
    # traci.start([sumoBinary, "-c", "../../sumo/test_cases/grid_33/grid_33.sumocfg"])

    sumo_interface = Connection("../../sumo/test_cases/grid_33/grid_33.sumocfg")
    # execute the TraCI control loop
    step = 0
    sumo_interface.extract_map()
    steps_pheromones = []

    gif_generator = GifPainter()

    while traci.simulation.getMinExpectedNumber() > 0:
        step += 1
        sumo_interface.simulate()
        sumo_interface.update_car_ids()
        sumo_interface.add_time_step()

        traffic_map = sumo_interface.map
        cars = sumo_interface.cars_id
        travel_time = sumo_interface.travel_time_sample


        print('==',step,'==')
        for id,car in cars.items():
            print(id, car.route, car.location.edge.id)
        estimator = FixedSpeedEstimator(cars, traffic_map)
        estimator.update(cars)
        pheromones = estimator.pheromones
        # if step == 68:
        #     """
        #     TODO: test code here
        #     active at 100ms
        #     """
        #
        #     # test code start============================================
        #     get_pheromone_timeIndex_fig(pheromones,max_time_period=10, step=step)
        #     for edge in traffic_map:
        #         if not edge.id.startswith(':'):
        #             print(edge.id,pheromones.get_pheromones(edge.id))
        if step%20==0:
            # steps_pheromones.append(pheromones)
            gif_generator.append_frame_weights(pheromones)
        if step==300:
            # get_step_pheromone_fig(steps_pheromones,start_step=100, stride= 1)
            gif_generator.print("test_s")
        if step ==200:
            gif_p = GifPainter(type=GifPainter.TYPE_PREDICTION)
            gif_p.append_all_weights(pheromones)
            gif_p.print("test_p")



    traci.close()
    sys.stdout.flush()
