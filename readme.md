# 2d Web Simulator

## This project contains two main component:
### The simulation client (The view on the browser) in the client folder
### The server that connects the web simulation to the rest of the system (node folder)

# Getting familiar
## Every folder has an inner readme that explains how to use that specific component, it is recommended that you first read these files before continuing

# Usage
## We have to start the main ROS app and the server before starting the client

### Main ROS app (follow the instructions in the example folder)
```
cd example
catkin_make
source devel/setup.bash
roslaunch launcher robot.launch
```

### Simulation server node (follow the instructions in the node folder)
```
cd node
catkin_make
./run.sh
```

### Finaly launch the client app (follow the instructions in the client folder)
### start file server to serve the index.html, the css and js files.
### Might need to refresh page between runs

# You can then send a start command to the robot as explained in the example folder readme.md and see the robot moving in the web interface

# Result

## Start and Change Algorithm 
<!-- <video width="630" height="300" src="./images/start&change.mp4"></video> -->

## Stop and Return 
<!-- <video width="630" height="300" src="./images/stop&return.mp4"></video> -->