# A-star-on-turtlebot2
Implemented a * algorithm with differential constraints ans simulated the robot motion in v-rep

please run obs.m first which creates obstacle space for the given map.

Then run astarturtle_differential.m which does a* algorithm with differential constraints.

When you run this file it prompts you to input start and goal locations, input accordingly. The first two values in the input are x and y, and (third value is theta set theta to zero in both start and endnode). Example [x y 0].

Next step, Please open rrl_map scene in the folder before running vrep_turtle.m.

Now run vrep_turtle.m, to check the simulation in vrep. After the robot reaches end position in vrep it pauses for 10 sec and simulation stops. Press ctrl+c in matlab command window to stop it after this.

The coordinate axis is set at bottom left corner of the map.
