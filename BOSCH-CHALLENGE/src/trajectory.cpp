#include "trajectory.h"

using namespace std;

Trajectory::Trajectory(Vehicle& car, const BehaviorType behavior){

  const double FRONT_BUFFER = 15.0;
  double target_s = 0.0;
  double target_v = car.saved_state_s.v;
  double start_v = car.saved_state_s.v;
  this->n = NUMBER_OF_POINTS;
  this->t = TRAVERSE_TIME;

  // get target states based on behavior s component

  if (behavior == BehaviorType::KEEPLANE) {

    double dv = 1.8;

    this->t = TRAVERSE_TIME_STRAIGHT;

    // move faster a little bit each time
    target_v = car.saved_state_s.v + dv;

    // it's dangerous, if we are too near and going at faster speed than in the car in front
    double not_safe = car.front_gap < FRONT_BUFFER && target_v > car.front_v;

    // if not safe let's go slower
    target_v = not_safe ?  target_v - 2.0 * dv : target_v;
    target_v = target_v < HARD_SPEED_LIMIT ? target_v : HARD_SPEED_LIMIT;

    if (car.lane == LaneType::RIGHT) {
      target_v = target_v < CHANGE_SPEED_LIMIT ? target_v : CHANGE_SPEED_LIMIT;
    }

    target_s =  car.saved_state_s.p + this->t * 0.5 * (car.saved_state_s.v + target_v);

  } else { // We're gonna changelane

    this->t = TRAVERSE_TIME;

    if (target_v > CHANGE_SPEED_LIMIT) {
      target_v = CHANGE_SPEED_LIMIT;
      this->t = TRAVERSE_TIME + 0.3;
    }

    cout << "LANECHANGE: (t, v0, vf) " << this->t << "---" << start_v << "---" << target_v << endl;

    target_s =  car.saved_state_s.p + this->t * 0.5 * (car.saved_state_s.v + target_v);
  }

  // target acceleration along the load is zero
  this->targetState_s = {target_s, target_v, 0.0};

  // get target d component state based on behavior
  // target speed and acceleration sideways of the road are both zero
  this->targetState_d = {car.get_target_d(behavior), 0.0, 0.0};

  // generate JMTs
  JMT jmt_s(car.saved_state_s, targetState_s, this->t);
  JMT jmt_d(car.saved_state_d, targetState_d, this->t);
  this->jmtPair.emplace_back(jmt_s);
  this->jmtPair.emplace_back(jmt_d);
  this->n = (this->t / TIME_INCREMENT);
}

JMT Trajectory::get_jmt_s() const {
  return jmtPair[0];
}

JMT Trajectory::get_jmt_d() const {
  return jmtPair[1];
}
