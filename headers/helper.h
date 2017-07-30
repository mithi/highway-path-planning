#ifndef HELPERS_H_
#define HELPERS_H_

#include <assert.h>
#include <vector>

//#include "jmt.h"


const double REALLY_BIG_NUMBER = 100000.0;

const double SPEEDLIMIT = 22.352; // 50mph in m/s
const double ACCELLIMIT = 10.0; // m/s^2


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
 * the simulator. Also holds an int n intended to store the number of (x, y) pairs
 */
struct XYPoints {
  std::vector<double> xs;
  std::vector<double> ys;
  int n;
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

#endif // HELPERS_H_
