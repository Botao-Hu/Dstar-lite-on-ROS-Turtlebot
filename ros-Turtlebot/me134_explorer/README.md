# me134_explorer 

This is a sample ROS node for students in CS/EE/ME 134 to clone and modify to explore the simulated space.

See [lab3_prelab.pdf](./lab3_prelab.pdf) for instructions on running in stage. Instructions for running on the real robot are below.

In addition to what's described in the lab3_prelab.pdf I have added command line options to the me134_explorer.py.
Included in those options (see below) is a --strategy option. This allows you to change the default strategy from FindClosestFrontier (which fails because it would crash the robot into a wall) to FindRandomEmptySpace (which just finds a random empty square of 2*safety_radius_m  meters by 2*safety_radius_m meters. You are expected to write an algorithm that explores the map more efficiently than FindRandomEmptySpace.

The global_cost_map doesn't seem to update regularly so I've turned off it's plotting by default. You're free to use it if you can figure out how to read the updates.

```
$ rosrun me134_explorer me134_explorer.py --help
usage: me134_explorer.py [-h]
                         [--strategy {FindClosestFrontier,FindRandomEmptySpace}]
                         [--safety_radius_m SAFETY_RADIUS_M]
                         [--initial_movement_m INITIAL_MOVEMENT_M]
                         [--plot_global_costmap PLOT_GLOBAL_COSTMAP]
                         [--plot_map PLOT_MAP]

optional arguments:
  -h, --help            show this help message and exit
  --strategy {FindClosestFrontier,FindRandomEmptySpace}
  --safety_radius_m SAFETY_RADIUS_M
                        default: 0.25
  --initial_movement_m INITIAL_MOVEMENT_M
                        default: -0.25
  --plot_global_costmap PLOT_GLOBAL_COSTMAP
                        default: 0
  --plot_map PLOT_MAP   default: 1
```

Pull requests welcome.

## Running me134_explorer.py on the real turtlebot

### Ensure time synchronization between the computers.
On the turtlebot's netbook and the controlling computer, connect to the internet and run:
```
BOTH> sudo ntpdate pool.ntp.org
```
Switch the network back to the local wireless/wired connection on both
machines. (NOTE to future TAs, we recommend running an ntp server on
the laptop (and/or the optitrack server) and having the netbook automatically sync with that server (probably using chrony).)

### Ensure that ROS_* environment variables are correct

You might look in the .bashrc files on both machines to ensure that ROS_MASTER_URI and ROS_HOSTNAME are set correctly.

```   
#For the normal wireless config on the controlling laptop it should be:
export ROS_MASTER_URI=http://192.168.1.57:11311
export ROS_HOSTNAME=192.168.1.3

#and on the turtlebot netbot:
export ROS_MASTER_URI=http://localhost:11311
export ROS_HOSTNAME=192.168.1.57
```

Note that each computer echos the current values of these variables when you run bash, so if they are correct you don't need to change them.

### Bring up the turtlebot and hokuyo_node on the netbook.    
On the controlling laptop, ssh into the turtlebot and run:
```
turtlebot> roslaunch turtlebot_bringup minimal.launch
```
In another terminal window or using tmux, terminator, etc run again on the turtlebot:
```
turtlebot> sudo chmod a+rw /dev/ttyACM0
```
Note to future TAs: Figure out how to have these permissions set correctly automatically. It's not that hard, but we're out of time.

Then run:
```
    turtlebot> rosrun hokuyo_node hokuyo_node
```
You're done running things on the robot now.

### Optitrack 
If you have optitrack available, on the controlling laptop, run
```
laptop> roslaunch vrpn_client_ros sample.launch server:=192.168.1.2
```

### launch explore_on_earth.launch
On the controlling laptop run:
```
laptop> roslaunch me134_explorer explore_on_earth.launch
```
This will start tf_setup, move_base, gmapping, and rviz with the proper config. (However displaying odom seems to crash rviz.)

### rosbag record
Now would be a good time to start recording a bag file using:
```
laptop> rosbag record -a -o lab3_yourname
```
The -o option sets a prefix for time stamped bag file. This means you can use this command more than once without worrying about overwriting files.

### me134_explorer.py
Now you should try running your algorithm. For testing purposes you could use the built in FindRandomEmptySpaces strategy:
```
laptop> rosrun me134_explorer me134_explorer.py --strategy=FindRandomEmptySpace
```
Alternatively if you need to modify some of the parameters you could try:
```
laptop> rosrun me134_explorer me134_explorer.py --initial_movement_m=-0.2 --strategy=FindRandomEmptySpace --safety_radius_m 0.3
```
If you want to run the algorithm without stopping to plot, you could try using the option --plot 0 to skip plotting.

When you're done kill the rosbag process with CTRL-c and shutdown the
other processes in the reverse order that you started them. If you
have plots open you may need to close them before CTRL-c will kill me134_explorer.py.