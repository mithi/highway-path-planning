# Introduction

This is my submission to the path planning project for term 3 of Udacity's self-driving car program.
The goal is to create a path planning pipeline that would smartly, safely, and comfortably navigate a virtual car around a virtual highway with other traffic. We are given a map of the highway, as well as sensor fusion and localization data about our car and nearby cars. We are supposed to give back a set of map points (x, y) that a perfect controller will execute every 0.02 seconds. Navigating safely and comfortably means we don't bump into other cars, we don't exceed the maximum speed, acceleration and jerk requirements. Navigating smartly means we change lanes when we have to.

# Links

-  [Video of this in action](https://www.youtube.com/watch?v=YoFUTnNaugQ)
-  [Model Documentation and Reflections](https://github.com/mithi/highway-path-planning/blob/master/docs/MODEL_DOCUMENT_V1.pdf)
- [Original project repository](https://github.com/udacity/CarND-Path-Planning-Project/blob/master/README.md)
- Original [Udacity ReadMe](https://github.com/mithi/highway-path-planning/blob/master/docs/UDACITY_README.MD)
- [Simulator Download Link](https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2)

# Project Goals Achieved

### The following criteria have been met.
- The car is able to drive at least 4.32 miles without incident.
- The car doesn't drive faster than the speed limit. Also the car isn't driving much slower than speed limit unless obstructed by traffic.
- Max Acceleration and Jerk are not exceeded. The car does not exceed a total acceleration of 10 m/s^2 and a jerk of 10 m/s^3.
- Car does not have collisions. The car must not come into contact with any of the other cars on the road.
- The car stays in its lane, except for the time between changing lanes. The car doesn't spend more than a 3 second length out side the lane lanes during changing lanes, and every other time the car stays inside one of the 3 lanes on the right hand side of the road.
- The car is able to change lanes. The car is able to smoothly change lanes when it makes sense to do so, such as when behind a slower moving car and an adjacent lane is clear of other traffic.


# How to use

- Download the [simulator](https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2)
and installed

- Clone this repo.
- Make a build directory in this directory `mkdir build && cd build`
- Compile: `cmake .. && make`
- Run it: `./path_planning.`

- Open the aforementioned simulator
- Choose the smallest resolution and fastest graphics quality of best results
- Click the path planning section
- Watch your automous car go :)
