#ifndef HELPERS_H_
#define HELPERS_H_

#include <vector>

// boundaries of acceptable speed of our vehicle
const double HARD_SPEED_LIMIT = 21.90; // 50mph in m/s is 22.352
const double CHANGE_SPEED_LIMIT = 21.6;

// Here's the duration period for each path plan we send to the controller
const double TIME_INCREMENT = 0.02;
const double TRAVERSE_TIME = 1.6;
const double TRAVERSE_TIME_STRAIGHT = 0.30;
const int NUMBER_OF_POINTS = int(TRAVERSE_TIME / TIME_INCREMENT);
const int NUMBER_OF_POINTS_STRAIGHT = int(TRAVERSE_TIME_STRAIGHT / TIME_INCREMENT);

const double REALLY_BIG_NUMBER = 1000000.0;
const double DUMMY_BACK = 20.0;

// the d value of each lane's center
const double LEFT_d = 2.45;
const double MID_d = 6.0;
const double RIGHT_d = 9.55;

// how much points left for the controller to perform
// before we start planning again
const int  PATH_SIZE_CUTOFF = 10;

// used as parameter in BehaviorPlanner::get_gap()
const double FROM_FRONT = 1.0;
const double FROM_BACK = -1.0;

const double mph_to_ms = 0.44704;

// Total road distance of the the highway loop
const double TRACK_DISTANCE = 5104.62105369568;
enum class LaneType {
  LEFT, MID, RIGHT, NONE, UNSPECIFIED
};

enum class BehaviorType {
  KEEPLANE, TURNRIGHT, TURNLEFT
};

/* State - stores three doubles p, v, a
 * intended to store position, velocity, and acceleration components in the s, or d axis
 */
struct State {
  double p;
  double v;
  double a;
};

/* XYPoints stores two vectors x, y which are the map coordinates to be passed to
 * the simulator. Also holds an int n intended to store the number of (x, y) pairs
 */
struct XYPoints {
  std::vector<double> xs;
  std::vector<double> ys;
  int n;
};

#endif // HELPERS_H_
