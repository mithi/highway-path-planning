#ifndef HELPERS_H_
#define HELPERS_H_

#include <assert.h>
#include <vector>

const double FROM_FRONT = 1.0;
const double FROM_BACK = 1.0;

const double REALLY_BIG_NUMBER = 1000000.0;

const double HARDSPEEDLIMIT = 22.352; // 50mph in m/s
const double SPEEDLIMIT = 22.0;

const double TIME_INCREMENT = 0.02;
const double TRAVERSE_TIME = 2.5;
const int NUMBER_OF_POINTS = int(TIME_INCREMENT * TRAVERSE_TIME);

const double FRONT_GAP_THRESH = 0.0; // 0, 5, 10, 15, 20
const double BACK_GAP_THRESH = 0.0; // 0, 5, 10
const double CHANGELANE_PENALTY = 0.0; // 0, 5

const int  PATH_SIZE_CUTOFF = 15;// 10, 15, 20

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

#endif // HELPERS_H_
