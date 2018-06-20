# Dstar-lite-on-ROS-Turtlebot
*Simulation: ME/CS 133b Robotics final project, Caltech, winter 2017, Contributor: Botao Hu, Yukai Liu, Guanya Shi.*  
*ROS & Turtlebot: CS/ME/EE 134 Autonomy final project, Caltech, spring 2018, Contributor: Botao Hu, Yukai Liu, Guanya Shi, Yan Wu, Yu-Wei Wu.  
(In alphebatical order)*

## 1. Simulation

Motion planning, which is a fundemental topic in algorithm and robotics, considers the problem where a robot needs to determine the shortest path from some starting point to the goal, while avoiding all the obstacles in the environment.

D* Lite, a simplification of D* algorithm, searches reversely from the goal and attempts to work back to the start, and uses current optimal path and heuristic estimations to greedily expand every node.

This project firstly implements D* lite algorithm based on python. In order to test the robot's performance, we also generates random mazes and record its path via MATLAB. See `./D_star_lite_simulation/readme.txt` for usage instructions.

Click the following screenshot to watch D* lite simulation video.

<p align="center">
<a href="http://www.youtube.com/watch?feature=player_embedded&v=h6H3n0BNXi8
" target="_blank"><img src="http://img.youtube.com/vi/h6H3n0BNXi8/maxresdefault.jpg" 
alt="IMAGE ALT TEXT HERE" width="480" height="240" border="10" /></a>
</p>

## 2. ROS & Turtlebot

We plugged D* lite planner as external packages into Robot Operating System, and launched with a Turtlebot in real practice.

Our algorithm and implementation worked well on accuracy and efficiency, in ROS simulator, simple area experiment and complex area experiment.

Click the following screenshot to watch the Turtlebot's performance in read world. **Second experiment video also included in the same channel**.

<p align="center">
<a href="http://www.youtube.com/watch?feature=player_embedded&v=ttc8bM89Alk
" target="_blank"><img src="http://img.youtube.com/vi/ttc8bM89Alk/0.jpg" 
alt="IMAGE ALT TEXT HERE" width="360" height="240" border="10" /></a>
</p>

## 3. Usage

One of the ROS package interface is adapted from [this repo](https://github.com/jaedlund/me134_explorer) with the same structure, but the contents are mostly changed.

### Global Planner Plugin

Copy `./ros-Turtlebot/d_star_lite` to `YOUR_WORKSPACE/src`. Generally, follow [this tutorial](http://wiki.ros.org/navigation/Tutorials/Writing%20A%20Global%20Path%20Planner%20As%20Plugin%20in%20ROS) to plug in a naive global planner that simply goes straight from start to goal. Several notes:

* C++ header file is provided as an interface, with cpp file implementation. You **must** implement methods provided by the super class.
* Remember to source your work space after registering your planner configurations.
```
cd catkin_ws
source devel/setup.bash
```
* Your default global planner is changed once you have done this step. Remember to recover it after use, in `move_base` node launch file.

### ROS setup

Copy `./ros-Turtlebot/me134_explorer` to `YOUR_WORKSPACE/src`. Follow the tutorial of the above repo to set up the environment. For simulation, you can
* Change the default map,
* Change the output size of `gmapping` node,
* Use navigate 2D in rviz to test new global planner.

### Run D* lite

Navigate to `./script` folder, and run `me134_explorer.py` while everything else in the environment is correctly running in terminals (Turtlebot, sensor, gmapping, etc). `d_star_lite.py` provides definition of the planner class, keep it. Sometimes you might run into map size issues, remember to change the input map array size according to your gmapping settings. You can turn off plot functionality if you find it annoying.

### Enjoy watching your Turtlebot!
