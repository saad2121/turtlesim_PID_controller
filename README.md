# turtlesim_PID_controller
This repository contains the implementation of PID controller on turtlesim for moving the turtle from start position and orientation to goal position and orientation. 
The model used is odometry based motion model which is generally used in differential drive robots. This model consist of 3 steps to reach the goal position and orientation which are as follows:
- Turn by delta rotation 1
- Move straight for delta translation
- turn by delta roation 2

![](https://github.com/saad2121/turtlesim_PID_controller/blob/master/results/odometry_model.PNG)

## Nodes
- ``` move.py``` - implementation to move the turtle in forward or backward direction (x direction) by given distance using constant linear velocity.

- ``` rotate.py``` - implementation of rotation by given angle (input - rotation angle) on the spot using constant angular velocity.

- ``` gotogoal.py``` - implementation to move from current position to goal position using linear velocity in ```x``` and angluar velocity in ```z``` direction (rotation about z) *NOTE: It is only of **x** and **y** and not the orientation and this implementation doesn't uses the odometry based motion model*.

- ```gotogoal_P.py ``` - implementation of *proportional controller* to move the turtle from current position ```(x, y, theta)``` to goal position ```(x', y' , theta')``` using odometry based motion model.

- ```gotogoal_PID.py ``` - implementation of *PID controller* to move the turtle from current position ```(x, y, theta)``` to goal position ```(x', y' , theta')``` using odometry based motion model.

## Usage 
**NOTE** - Ensure that ROS and the ROS turtlesim package is installed in your machine.

In a new terminal window, start the ROS master server :

``` 
roscore
```

Open new terminal (Ctrl + Alt + t) and start turtlesim simulator

``` 
rosrun turtlesim turtlesim_node 
```

**NOTE** - ensure that this repo is in ```src``` folder of catkin workspace (Assume name of workspace is catkin_ws)
in new terminal type
``` 
cd ~/catkin_ws
catkin_make
soure ./devel/setup.bash 
```

