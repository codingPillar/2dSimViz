# 2d Simulation Environment Server

## This is the server node that acts as a bridge between the web simulation environment and the rest of the ROS system.

## The web environment is responsible for sending odom and lidar data and moves the object according to the cmd_vel values that it receives

## The role for this node is to subscribe to the cmd_vel topic and send the received data to the web environment and to publish the odom and lidar data to the ROS subsystem. Is also publishes the transform from the base_link coordinate system to the odom coordinate system (Needed by slam_gmapping node). This node could be extented to work quite easily with robot_state_publisher and robot_joint_states nodes but since it is a 2d sim, it is easier to model everything as a single link. 

# GETTING STARTED

## We have 2 supported build systems, make and cmake (through catkin_make).

### With Make
```
cd src/2dVizServer
make
make start
```
### This will build an executable named node in folder src/2dVizServer and start the server on port 50601 (port number that is assumed by the client web interface)

### With cmake
```
catkin_make
./run.sh
```
### The script run.sh is equivalent to calling make start from previous method. Starts the server node on port 50601

# Tests
## Tests are only supported with make for now (gtest is needed as a dependency)
```
cd src/2dVizServer
make tests
./tests
```

# Limitation

## Support for odom publishing is only for x and y position and rotation arround z axis (Normal 2D rotations)

# TODO

## Automatically Launch chrome window when starting server and destroying that same window when stopping the server
