# parametric_trajectory_generator
The repository is a C++ implementation of trajectory generation algorithm introduced in Coursera course - Motion Planning for Self Driving Cars.

The trajectory generator will generate a list of waypoint for the vehicle controller to folllow according to desired target vehicle pose. 

# How to run
## Dependicies
* Eigen3

## Building
The repository uses CMake for building. 
The program is tested under Ubuntu 16.04.
### Step to build
clone the repository
in the repostiroy folder, create a build folder by
```C
mkdir build
cd build
```
in the build folder, create the Makefile by 
```C
cmake ..
```
then, in the build folder, compile the program by
```C
make
```
A library file __libligTrajectoryGenerator.so__ will show in the build folder

You can run the demo program in the python source folder. The python demo program will call the library and plot the generated trajectory.

A simple example is also available in the cpp source file to show how to use the trajectory generator.

# Introduction
## Assumptions
*   The vehicle is at origin with zero heading angle. <br>(x, y, theta) = (0, 0, 0)</br>

## Input
*   Desired target vehicle pose <br> (x_target, y_target, theta_target) </br>

## Output
*   A list of waypoints sampled from computed spline trajectory for the vehicle to follow

## Parameters
*   Max iteration for optimization process
*   Terminate condition to stop optimization iteration
*   Pertubation value for jacobian calculation
*   Sample resolution of the output waypoint list (meters)   
*   spline parameters

## Error detection and handling
*   An empty list will be returned if
    *   the generator can not compute a valid trajectory 
    *   the generator can not converge in defined iteration

*   Need to do error handling if following happens
    *   optimization failed, an NAN happend   
    *   optimization produced an invalid result, eg, s is negative

## Workflow
*   The trajectory generator workflow is as follow. <br>
    1.  initial spline parameters by target vehicle pose 
    2.  compute motion update by one-shot method
    3.  check if iteration stop criteria is met or not (eg, reach max iteration, target state computed by spline converges)
        *   if no, go to step 4
        *   if stop criteria is met and is converged, go to step 10
        *   if stop criteria is met and is not converged, to to step 11  
    4.  compute jacobian by one-shot method according to the defined pertubation value and current spline parameters
    5.  compute predicted state by one-shot method with current spline parameters
    6.  compute error between predicted target state and desired target state
    7.  compute update value of spline parameters by jacobian and state error
    8.  update spline parameters
    9.  back to step 2
    10. sample waypoints on converged spline by one-shote method according to defined sampling resolution and return the sampled waypoint list
    11. return empty waypoint list

## Python implementation
A python implementation can be found in [pyTrajectoryGenerator](https://github.com/gn02596703/pyTrajectoryGenerator)

