# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   
### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases).

### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving below the 50 MPH speed limit. The car's uses idealised but delayed localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car tries to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 50 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake ../src && make`  
   To create an Eclipse project, run `cmake -G"Eclipse CDT4 - Unix Makefiles" ../src `
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

I changed the location of the CMakeLists.txt file following a warning message from cmake:
    CMake Warning in CMakeLists.txt:
    - The build directory is a subdirectory of the source directory.

    - This is not supported well by Eclipse.  It is strongly recommended to use a
      build directory which is a sibling of the source directory.



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

## Details

This car controller is a simple prototype for a highway controller.
Its implementation is both simple and naive and I see many areas that would benefit from improvement.
For example, a more robust implementation would implement a probalistic and predictive behavioural model of the other vehicles on the road and base the driving decisions on this predictive model.
Currently the model only considers a first-order estimation of the future state of the vehicles in Frenet coordinates. 

Now, let's describe the implementation of the controller.

### main.cpp
Load up map values for waypoint's x,y,s and d normalized normal vectors read from a file.
The RoadGeometry object handles waypoint map and handles the road geometry.

The file instantiate double buffer to save a first few points. The data is logged using a Double buffering thread that runs concurrently to the main thread.

The main function instanciates the DrivingState class. This class is responsible for taking driving decisions depending on other vehicles in vicinity.

The main class uses a WebSocket callback to communicate with the simulator to provide the future trajectory points and to retrieve Sensor Fusion data.

### trajectory.cpp
Functions:
- ideal_trajectory: this function implements an ideal trajectory that was useful for debugging trajectory closing-loop issues.
- my_trajectory: this function implements a jerk minimized trajectory parametrized by desired speed and accelerations.
This function calculates and control the Self Driving vehicle speed and lane position using a Jerk minimizing function to control the vehicle speed. It uses the previously calculated for implementing a continous path. The Jerk minimization is computed in the Frenet coordinate system.
The function also tries to estimate the time lag between the previously sent datapoints and the current step.
my_trajectory calls functions in optim_jmt.cpp

### optim_jmt.cpp
Functions:
- optim_jmt_affine: this function optimises the jerk for the s coordinate in the Frenet system. 
Optimize final distance position s start and s final so that the jerk is defined by an affine function. This type of affine jerk seems to reduce the envelope of the jerk and the acceleration. The speed tends do not overshoot in this configuration. This was found in my quick experiments but would need to be proven formally.
The routine uses a concept of virtual acceleration. The equivalent virtual acceleration corresponds to
the equivalent acceleration that would satisfy the speed constraints.

- optim_jmt_quadratic : this function optimises the jerk for the d coordinate in the Frenet system is the quadratic Jerk version of optim_jmt_affine. The function solves for a minimum quadratic Jerk polynomial and uses a concept of virtual speed, similar to the virtual acceleration concept used for the optim_jmt_affine.


### jmt.cpp
Functions:
- JMT and  JMT_affine: : uses the C++ Matrix calculation Eigen library to compute the functions as described in optim_jmt.cpp

### planner.cpp
Implements the class DrivingState. This class decides the finite state machine that computes the driving decisions that the vehicle takes on the highway as a function of the other driving agents.
The main metric used to evaluate the cost of the decision is the estimated time to collision to vehicles in vicinity.

### utils.cpp
Implements the vehicles position points logger using a concurrent thread that uses double buffering.
[Example of a glitchy speed profile debugged thanks to my double buffered logger](script/need_to_adjust_new_points.png)


### points.cpp
Implement some simple data structures to perform some calculations on waypoints.

## Tips

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.

---

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

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!


## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).

