#ifndef HELPERS_H_
#define HELPERS_H_

#include <vector>

// Here's the duration period for each path plan we send to the controller
const double TIME_INCREMENT = 0.02;
const double TRAVERSE_TIME = 2.0;
const int NUMBER_OF_POINTS = int(TRAVERSE_TIME / TIME_INCREMENT);

const double REALLY_BIG_NUMBER = 1000000.0;

// the d value of each lane's center
const double LEFT_d = 2.2;
const double MID_d = 6.0;
const double RIGHT_d = 9.8;

// how much points left for the controller to perform
// before we start planning again
const int  PATH_SIZE_CUTOFF = 10;

// used as parameter in BehaviorPlanner::get_gap()
const double FROM_FRONT = 1.0;
const double FROM_BACK = -1.0;

// Total road distance of the the highway loop
const double TRACK_DISTANCE = 6945.554;

// boundaries of acceptable speed of our vehicle
const double HARD_SPEED_LIMIT = 22.352; // 50mph in m/s
const double SPEED_LIMIT = 20.75;
const double MIN_SPEED = 15.0;

// if the gap is less than this we consider it unsafe to turn
const double FRONT_GAP_THRESH = 25.0;
const double BACK_GAP_THRESH = 10.0;

// This is the buffers we want against the leading front vehicle
// for safety so we don't collide with the vehicle right in front of us
const double FRONT_BUFFER = FRONT_GAP_THRESH + 10.0;
const double DISTANCE_BUFFER = 5.0;
const double SPEED_BUFFER = 6.0;

// Parameters than can be tweaked which affects the cost of each behavior
const double MIDLANE_REWARD_FACTOR = 0.35; //must be 0 < x < 1
const double BACK_GAP_FACTOR = 0.4; // must be less than FRONT_GAP_FACTOR
const double FRONT_GAP_FACTOR = 1.0;
const double TURN_PENALTY_FACTOR = 1.4; // must be x > 1

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
