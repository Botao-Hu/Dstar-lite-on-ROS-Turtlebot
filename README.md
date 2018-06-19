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

Our algorithm and implementation worked well on accuracy and efficiency, on ROS simulator, simple area experiment and complex area experiment.

Click the following screenshot to watch the Turtlebot's performance in read world. **Second experiment video also included in the same channel**.

<p align="center">
<a href="http://www.youtube.com/watch?feature=player_embedded&v=ttc8bM89Alk
" target="_blank"><img src="http://img.youtube.com/vi/ttc8bM89Alk/0.jpg" 
alt="IMAGE ALT TEXT HERE" width="360" height="240" border="10" /></a>
</p>

## 3. Usage

The ROS package interface is adapted from https://github.com/jaedlund/me134_explorer with the same structure, but the contents are mostly changed.

### Global Planner Plugin
* C++ file
