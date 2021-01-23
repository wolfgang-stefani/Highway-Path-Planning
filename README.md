# Highway-Path-Planning
Path planner that creates smooth, safe trajectories for an autonomous vehicle to follow along a 3 lane highway with traffic. The path planner is able to keep inside its lane, avoid hitting other cars, and pass slower moving traffic all by using localization, sensor fusion, and map data.

Point Paths
The path planner outputs a list of x and y global map coordinates. Each pair of x and y coordinates is a point, and all of the points together form a trajectory.

The highway has 6 lanes total - 3 heading in each direction. Each lane is 4 m wide and the car should only ever be in one of the 3 lanes on the right-hand side. The car should always be inside a lane unless doing a lane change.

The first waypoint has an s value of 0 because it is the starting point.
