# The Project
This project designed to act as a proof of concept for package transfer between trucks. For this proof of concept we are using two turtlebots to trasfer a golf ball to each other while moving. The algorithms will be written in a scalable manner.

# MVP 11/27

1. Find a meeting point in 2D Space
2. Use move_base to get there and perform a stationary transfer
3. Use robot localization to move robot A to bump into B and transfer the package
4. Perform the exchange
5. Use move_base to get to the final destination

# Extension 1 *** MOST IMPORTANT ***

- Move A to be in view of B's AR Tag and have it be able to transfer the package to B while it is moving toward its final destination.

# Extension 2 

- Three turtlebots!!! 
- Make the above MVP into a service capable of being executed multiple times and being updated in real-time.

# Launch startup

```bash
roslaunch move_turtlebot init.launch
```

# State Machine design

- event input gathering
- state machine enter exit functions
- compute new goals
- send new goals

- Take in topics from amcl_pose and move_base navigation complete info
- Output to control a new CMD_VEL_MUX to control node outputs

[http://wiki.ros.org/turtlebot_navigation/Tutorials/Setup%20the%20Navigation%20Stack%20for%20TurtleBot](http://wiki.ros.org/turtlebot_navigation/Tutorials/Setup%20the%20Navigation%20Stack%20for%20TurtleBot)
