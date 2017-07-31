#ifndef HELPERS_H_
#define HELPERS_H_

#include <vector>

const double FROM_FRONT = 1.0;
const double FROM_BACK = -1.0;

const double REALLY_BIG_NUMBER = 1000000.0;

const double TRACK_DISTANCE = 6945.554;

const double HARD_SPEED_LIMIT = 22.352; // 50mph in m/s
const double SPEED_LIMIT = 21.0;

const double TIME_INCREMENT = 0.02;
const double TRAVERSE_TIME = 2.0;
const int NUMBER_OF_POINTS = int(TRAVERSE_TIME / TIME_INCREMENT);

const double FRONT_GAP_THRESH = 30.0;
const double BACK_GAP_THRESH = 15.0;

const double SPEED_BUFFER = 5.0;

const double MIDLANE_REWARD_FACTOR = 0.25; //must be 0 < x < 1
const double BACK_GAP_FACTOR = 0.5;
const double FRONT_GAP_FACTOR = 1.0;
const double TURN_PENALTY_FACTOR = 1.25; // must be x > 1

const int  PATH_SIZE_CUTOFF = 10;

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
