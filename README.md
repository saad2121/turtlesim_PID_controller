# turtlesim_PID_controller
This repository contains the implementation of PID controller on turtlesim for moving the turtle from start position and orientation to goal position and orientation. 
The model used is odometry based motion model which is generally used in differential drive robots. This model consist of 3 steps to reach the goal position and orientation which is as follows:
- Turn by delta rotation 1
- Move straight for delta translation
- turn by delta roation 2
