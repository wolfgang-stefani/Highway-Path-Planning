# Highway-Path-Planning
Path planner that creates smooth, safe trajectories for an autonomous vehicle to follow along a 3 lane highway with traffic. The path planner is able to keep inside its lane, avoid hitting other cars, and pass slower moving traffic all by using localization, sensor fusion, and map data.


![](readme_data/Highway_Driving_Sim.gif)

## Description
The car safely navigates around a virtual highway with other traffic that is driving +-10 MPH off the 50 MPH speed limit. The highway has 6 lanes total - 3 heading in each direction. Each lane is 4 m wide and the car should only ever be in one of the 3 lanes on the right-hand side. The path planner is provided the car's localization and sensor fusion data. There is also a sparse map list of waypoints around the highway. The car goes as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible. Other cars will try to change lanes too. The car avoids hitting other cars at all cost and is driving inside of the marked road lanes at all times, unless going from one lane to another. Also the car does not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3 so that humans feel comfortable inside the car.

---
## Point Paths
The path planner outputs a list of x and y global map coordinates. Each pair of x and y coordinates is a point, and all of the points together form a trajectory. The first waypoint has an s value of 0 because it is the starting point.

The map of the highway is in `data/highway_map.txt`
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around. So the frenet s value, distance along the road, goes from 0 to 6945.554.

## Speed, Acceleration and Jerk

The car uses a perfect controller and will visit every (x,y) point it receives in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing between the points determines the speed of the car. The vector going from one point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner receives do not have a total acceleration that goes over 10 m/s^2, also the jerk does not go over 10 m/s^3.

## Spline function

For trajectory generation, the spline function is used instead of polynomial trajectory generation. One of the reasons is it is simple to use and requires no dependencies.
This was a really helpful resource for doing this project. The spline function can be found here: http://kluge.in-chemnitz.de/opensource/spline/

## Class definitions in the Code

Two classes are defined, one is `Vehicle` and one is `List_Vehicle`.

`Vehicle` contains or calculates all the necessary data to describe an object of the class car like its **position** (in frenet coordinates s and d), **id**, **lane** and **speed**. Furthermore the class provides two functions, one to **calculate the speed** in dependence of lateral + logitudinal speed and one to **predict the future position** in dependence of the speed calculated before.

`List_Vehicle`contains `vector<Vehicle>`and all the data related to the ego vehicle like the speed `ego_speed` or the predicted position `ego_future_s`. All the functions that process sensor readings are in this class. Also cost function calculations are included.

---

## Simulator
You can download the Simulator here: [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).  

To run the simulator on Mac/Linux, first make the binary file executable with the following command:
```shell
sudo chmod u+x {simulator_file_name}
```
## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program:

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

## Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
