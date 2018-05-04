
# PareCity User Manual
## Software Description
This project aims to propose a novel multi-agent dynamic routing algorithm aiming at minimising the expected travel time of each agent in the network in this system. We build this independent programme that can interface to the traffic simulation environment - SUMO. The main tasks of this program include training a model based on our algorithm as well as run a traffic flow simulation based on our algorithm and baselines. In addition, this program is aimed at researchers and mainly used for academic purpose.

## Hardware Requirements
To run this program on your personal computers, there are minimum hardware requirements for your computer.
* RAM
 * The suitable RAM usage should be less than 4 GB. Since we will run the demo and train the model on a common PC.

## Software Requirements
To run this program on your personal computers, following software are required and should be installed on your computer before using our program.

### 1. Python Packages
* tensorflow
* pandas
* matplotlib
* imageio
* numpy

Use `pip install` to install these packages.

### 2. Traffic simulation environment: "Simulation of Urban MObility(SUMO)"
Download the windows version in SUMO's official site(http://www.dlr.de/ts/en/desktopdefault.aspx/tabid-9883/16931_read-41000/).
Next is to run the installer to set up SUMO on your computer.

## How to use PareCity
In terms of parameter setting, you can set the following parameter directly in terminal or add it into configuration settings and run main.py(see following example).
<div align="center">
<img src="https://raw.githubusercontent.com/jycwy/img/master/manual_img/para_configuration.png" height="500px" alt="parameter setting based on PyCharm" >
</div>


### 1. Train a model
Set `--mode=train`, the program will train a specific model.
Example `python main.py --mode=train --model=grid0 --map=grid --gui=True`

* The `--model` is a required parameter.
    * The program will save the trained model and summary of training progress under
the models' folder with the name `--model` parameter specified.
    * The summary of the training process is a CSV file, which records the number of epochs, the average training loss, the average travel time in this epoch and the time an epoch finishes.
    * Please also note that if such model file already exists in the models' folder. The program will automatically continue the training.
    If one wants to start a new training, he should use a new model name.
* The `--map` specified the name of the simulation set which includes both the road network and configuration of
traffic low. Can be either of "grid" or "liverpool".
    * The default setting is "grid". It is a 3 by 3 gridded network.
    * The "liverpool" is a part of map of liverpool.
    <div align="center"><img src="https://raw.githubusercontent.com/jycwy/img/master/manual_img/liverpool.png" height="350px" alt="Liverpool map" ></div>
* The `--gui` is used to control if the sumo-gui is used. The default one is False.
If set `--gui=True`, then it will pop out SUMO GUI and you need to press the "Run" button in SUMO GUI to start training process. You are able to use mouse wheel to zoom the map.
<div align="center">
<img src="https://raw.githubusercontent.com/jycwy/img/master/manual_img/Sumo1.png" height="300px" alt="Run SUMO" >
<img src="https://raw.githubusercontent.com/jycwy/img/master/manual_img/Sumo2.png" height="300px" alt="SUMO GUI" >
</div>

### 2. Run a Simulation
Set `--mode=simulate`, the program run a simulation of traffic flow using specific routing algorithm.

* The `--router` indicates the algorithm that each vehicle in the simulation will use to reach its destination.
  * If `--router=astar` will use traditional astar to schedule a route, minimising the total route length.
  * `--router=nnrouter` means the routing on the time-weighted graph will be applied, where the travel time estimation is based on the deep leaning and pheromone mechanism.
    * Notably, the `--model` parameter should be set as a pre-trained model in this case.
  * `--router=arstar` ARstar is an improved model for astar used for multi-agent routing. It will automatically push the agent on the same edge away from each other. This is an implementation of https://pdfs.semanticscholar.org/bd3c/3f4d961cec6b33d43c201d789116a906448e.pdf.
* `--result` summary output directory for this simulation.
* `--map` is the same to that in training mode.
* `--gui` is the same to that in training mode.

Notice that every time running the simulation, there will be file called `visualization.gif` generated to show the pheromone change duration duration the simulation.
![visualization](https://user-images.githubusercontent.com/18574971/39634190-0d914c56-4fb2-11e8-8feb-e93043243dea.gif)



### 3. Plot
Set `--mode=plot`, can be used to visualize the training process.
For example, `python main.py --mode=plot --model=girdCross`

![gridcrosstrain](https://user-images.githubusercontent.com/15139574/39326156-06a8d0c0-498c-11e8-9714-0d0f7b1ce7aa.png)
