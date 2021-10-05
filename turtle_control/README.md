# Turtle Control Package

Author and Maintainer: **Ryan King-Shepard**

## **Description**
The turtle control package sends a turtle in turtlesim on a journey to follow a waypoint path.

This package contains:

- nodes:
    1. `setup` ~ Provides `draw` service to clear turtlesim and draw waypoints
    2. `translate` ~ converts 2d velocity commands into 3d commands of turtlesim
    3. `follow` ~ Provides `restart` service to call `draw` and then begin waypoint following
- msg:
    * `TurtleVelocity.msg` ~ 2D velocity command
- srv:
    * `Start.srv` ~ Request a staring location x, y and receive the distance the turtle will     travel along waypoint path. 
- config:
    * `waypoint.yaml` ~ Definitions of ros params used by service. Edit waypoint path here. 
- launch:
    * `run_waypoints.launch` ~ Launch file to start all nodes
- `waypoint.bag` 
    * rosbag file of cmd_vel topic for testing
- `package.xml`
    * Package configuration
- `CMakeLists.tx`
    * Build configuration
- `README.md`
    * Hello there!


## **Dependencies and Installation**

### *ROS Dependencies*
This package was developed and tested in ros-noetic. Turtle control is designed and should be built using catkin. This package also requires the turtlesim package to run. This should usually come with your ros installation, but to check this run: 
```bash
source /opt/ros/{ros-version}/setup.bash
rosrun turtlesim turtlesim_node
```
This should result a little screen with a turtle in the center. If the window doesn't pop up, refer to [ROS wiki](http://wiki.ros.org) to install turtlesim on your machine.

### *Python Dependencies*
This package does not use any third-party libraries outside of rospy and turtlesim; both should be included in the initial ROS install. This node was developed and test in Python 3 (specfically 3-8-10)

## **Execution**
To run the full suite and see a turtle follow a set of points, open a terminal and run the `run_waypoints.launch` file. 
```bash
roslaunch turtle_control run_waypoints.launch
```
If this command doesn't work, ensure you have ran `catkin_make` and `source devel/setup.bash` in your workspace. A successful command should have a turtlesim window pop up. You can also check by running `rosnode list` and seeing the following active nodes:
```bash
/follow
/rosout
/setup
/translate
/turtlesim_node
```
To start waypoint following, call the `restart` service with an x-y coordinate. **NOTE**: x, y should be in the range [0, 11]; if a point is given out of bounds, the service will reject it. 

Example: Starting at point(3,5)
```bash
rosservice call /restart "x: 3.0 y: 5.0"
``` 

The turtle should now draw the waypoint path, teleport the starting point, and begin following the path. To edit the path, edit the list in `config/waypoint.yaml` next to the `waypoints` parameter. 

To only use the `draw` service provided by the `setup` node, you can run the node and then call the `draw` service in a different terminal.

Terminal A
```bash
rosrun turtle_control setup
```

Termainal B
```bash
# /draw requires no arguments to run
rosservice call /draw "{}" 
```

