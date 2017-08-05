#include "trajectory.h"

Trajectory::Trajectory(Vehicle& car, const BehaviorType behavior){

  // get target states based on behavior s component
  double target_s = car.saved_state_s.p + TRAVERSE_TIME * car.saved_state_s.v;
  double target_v = car.saved_state_s.v;

  if (behavior == BehaviorType::KEEPLANE) {

    // If the car in front is going fast or we are very far from it anyway, go as fast as we can
    // Else let's go a notch slower than the car in front
    bool safe = (car.front_v > SPEED_LIMIT) || (car.front_gap > FRONT_BUFFER);
    target_v =  safe ? SPEED_LIMIT : (car.front_v - SPEED_BUFFER);
    
    // But if the car in front is too slow, let's go a little faster
    target_v = target_v > MIN_SPEED ? target_v : MIN_SPEED;
    
    // Estimate a safe target distance based on our selected speed
    target_s =  car.saved_state_s.p + TRAVERSE_TIME * 0.5 * (car.saved_state_s.v + target_v);
  }
  
  // target acceleration along the load is zero
  this->targetState_s = {target_s, target_v, 0.0};

  // get target d component state based on behavior
  // target speed and acceleration sideways of the road are both zero
  this->targetState_d = {car.get_target_d(behavior), 0.0, 0.0};

  // generate JMTs
  JMT jmt_s(car.saved_state_s, targetState_s, TRAVERSE_TIME);
  JMT jmt_d(car.saved_state_d, targetState_d, TRAVERSE_TIME);
  this->jmtPair.emplace_back(jmt_s);
  this->jmtPair.emplace_back(jmt_d);
}

JMT Trajectory::get_jmt_s() const {
  return jmtPair[0];
}

JMT Trajectory::get_jmt_d() const {
  return jmtPair[1];
}
