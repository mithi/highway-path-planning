#include "behaviorplanner.h"

BehaviorPlanner::BehaviorPlanner() {}

BehaviorType BehaviorPlanner::get(Vehicle& myCar, std::vector<Vehicle>& otherCars) {

  myCar.front_gap = this->get_gap(myCar, otherCars, myCar.lane, FROM_FRONT);

  const double frontleft = this->get_gap(myCar, otherCars, myCar.lane_at_left, FROM_FRONT);
  const double backleft = get_gap(myCar, otherCars, myCar.lane_at_left, FROM_BACK);
  const double frontright = this->get_gap(myCar, otherCars, myCar.lane_at_right, FROM_FRONT);
  const double backright = this->get_gap(myCar, otherCars, myCar.lane_at_right, FROM_BACK);

  const double left_cost = this->get_cost(frontleft, frontright, myCar.lane_at_left);
  const double right_cost = this->get_cost(frontright, backright, myCar.lane_at_left);

  const double straight_cost = this->get_cost(myCar.front_gap);

  if (straight_cost < left_cost && straight_cost < right_cost){
    return BehaviorType::KEEPLANE;
  }

  if (right_cost < straight_cost && right_cost < left_cost) {
      return BehaviorType::TURNRIGHT;
  }

  return BehaviorType::TURNLEFT;
}

double BehaviorPlanner::get_cost(const double front_gap, const double back_gap, const LaneType lane) const {

  if (lane == LaneType::NONE || lane == LaneType::UNSPECIFIED) {
    return REALLY_BIG_NUMBER;
  }

  const double effective_front_gap = front_gap - CHANGE_LANE_PENALTY;
  const double effective_back_gap = back_gap - CHANGE_LANE_PENALTY;

  if (effective_front_gap < FRONT_GAP_THRESH && effective_back_gap < BACK_GAP_THRESH) {
    return REALLY_BIG_NUMBER;
  }

  return (1.0 / effective_front_gap + 0.5 / effective_back_gap);
}


double BehaviorPlanner::get_cost(const double gap) const {

  if (gap < FRONT_GAP_THRESH) {
    return REALLY_BIG_NUMBER;
  }

  return 1.0 / gap;
}

double BehaviorPlanner::get_gap(
  const Vehicle &myCar, const std::vector<Vehicle>& otherCars, const LaneType lane_type, const double where) const {

  double smallest_gap = REALLY_BIG_NUMBER;

  for(auto &otherCar: otherCars){

    double gap = (otherCar.s - myCar.s) * where;

    if (otherCar.lane == lane_type && gap > 0.0 && gap < smallest_gap) {
      smallest_gap = gap;
    }
  }

  return smallest_gap;
}
