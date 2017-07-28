#ifndef HELPERS_H_
#define HELPERS_H_

#include <assert.h>
#include <vector>

//#include "jmt.h"

const double REALLY_BIG_NUMBER = 100000.0;
const int NEW_PATH_SIZE = 50;
const int KEEP_PATH_SIZE = 10;
const int LAG = 10;
const double TURN_DURATION = 5.0;
const double KEEP_DURATION = 1.0;
const double SPEEDLIMIT = 22.352; // 50mph in m/s
const double ACCELLIMIT = 10.0; // m/s^2
const double FRONTGAP_THRESH = 40.0;
const double BACKGAP_THRESH = 30.0;
const double GAP_SWITCH_COST = 0.0;
const double LARGE_GAP_THRESH = 70.0;
const double BUFFER_DISTANCE = 0.0;
const double BUFFER_SPEED = 0.0;
const double BUFFER_ACCEL = 0.0;
const double DELTA_ACCEL = 0.0;

enum class LaneType {
  LEFT, MID, RIGHT, NONE, UNSPECIFIED
};

enum class BehaviorType {
  KEEPLANE, TURNRIGHT, TURNLEFT
};

/* State - stores three doubles p, v, a
 * intended to store position, velocity, and acceleration components
 * in the s, or d axis
 */
struct State {
  double p;
  double v;
  double a;
};

/* XYPoints stores two vectors x, y which are the map coordinates to be passed to
 * the simulator. Also holds double n intended to store the number of (x, y) pairs
 */
struct XYPoints {
  std::vector<double> x;
  std::vector<double> y;
  double n;
};

/* StatePair stores two State objects
 *
 */
struct StatePair {
  State s_state;
  State d__state;
};

/* JMTPair stores two JMT objects
 *
 */
 /*
struct JMTPair {
  JMT s_jmt;
  JMT d_jmt;
};
*/


LaneType convert_d_to_lane(const double d){

  LaneType lane = LaneType::NONE;

  if (d > 0.0 && d < 4.0) {
    lane = LaneType::LEFT;
  } else if (d > 4.0 && d < 8.0) {
    lane = LaneType::MID;
  } else if (d > 8.0 && d < 12.0) {
    lane = LaneType::RIGHT;
  }

  return lane;
};


double convert_lane_to_d(const LaneType l){

  double d = -1;

  if (l == LaneType::LEFT) {
    d = 2.0;
  } else if (l == LaneType::MID) {
    d = 4.0;
  } else if (l == LaneType::MID) {
    d = 6.0;
  }

  return d;
}

#endif // HELPERS_H_
