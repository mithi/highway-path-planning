#include "behaviorplanner.h"

using namespace std;

BehaviorPlanner::BehaviorPlanner() {}

BehaviorType BehaviorPlanner::update(Vehicle& myCar, std::vector<Vehicle>& otherCars) {

  myCar.front_gap = this->get_gap(myCar, otherCars, myCar.lane, FROM_FRONT);
  myCar.front_v = this->current_front_v;
  myCar.front_s = this->current_front_s;

  cout << "LOOKING LEFT..." << endl;
  const double frontleft = this->get_gap(myCar, otherCars, myCar.lane_at_left, FROM_FRONT);
  const double backleft = get_gap(myCar, otherCars, myCar.lane_at_left, FROM_BACK);

  cout << "--> frontleft gap: " << frontleft << "vs. front straight gap:" << myCar.front_gap << endl;
  cout << "-->  backleft gap: " << backleft << endl;

  cout << "LOOKING RIGHT..." << endl;
  const double frontright = this->get_gap(myCar, otherCars, myCar.lane_at_right, FROM_FRONT);
  const double backright = this->get_gap(myCar, otherCars, myCar.lane_at_right, FROM_BACK);

  cout << "--> frontright gap: " << frontright << "vs. front straight gap:" << myCar.front_gap << endl;
  cout << "-->  backright gap: " << backright << endl;

  const double left_cost = this->get_cost(frontleft, frontright, myCar.lane_at_left);
  const double right_cost = this->get_cost(frontright, backright, myCar.lane_at_right);

  const double straight_cost = this->get_cost(myCar.front_gap);

  if (left_cost < straight_cost && left_cost < right_cost){
    cout << "TURN LEFT: " << left_cost << " < " << straight_cost << endl;
    cout << "-->" << endl;
    return BehaviorType::TURNLEFT;
  }

  if (right_cost < straight_cost && right_cost < left_cost) {
    cout << "TURN RIGHT: " << right_cost << " < " << straight_cost << endl;
    cout << "-->" << endl;
    return BehaviorType::TURNRIGHT;
  }

  cout << "KEEP LANE. costs: straight vs left vs right:" << endl;
  cout << "--> " << straight_cost << " . " << left_cost << " . " << right_cost << endl;
  return BehaviorType::KEEPLANE;
}

double BehaviorPlanner::get_cost(const double front_gap, const double back_gap, const LaneType lane) const {

  if (lane == LaneType::NONE || lane == LaneType::UNSPECIFIED) {
    return REALLY_BIG_NUMBER;
  }

  const double effective_front_gap = front_gap - CHANGE_LANE_PENALTY;
  const double effective_back_gap = back_gap - CHANGE_LANE_PENALTY;

  if (effective_front_gap < FRONT_GAP_THRESH || effective_back_gap < BACK_GAP_THRESH) {
    cout << "--> There's no space to turn!" << endl;
    cout << "--> front gap: " << front_gap << " back gap: " << back_gap << endl;
    return REALLY_BIG_NUMBER;
  } else {
    cout << "-->" << endl;
    cout << "-->" << endl;
  }

  return (1.0 / effective_front_gap + 0.5 / effective_back_gap);
}


double BehaviorPlanner::get_cost(const double gap) const {

  if (gap < FRONT_GAP_THRESH) {
    cout << "--> We are too near the front vehicle we must turn! gap:  " << gap << endl;
    return REALLY_BIG_NUMBER;
  } else {
    cout << " " << endl;
  }

  return 1.0 / gap;
}

double BehaviorPlanner::get_gap(
  const Vehicle &myCar, const std::vector<Vehicle>& otherCars, const LaneType lane_type, const double where) {

  if (lane_type == LaneType::NONE || lane_type == LaneType::UNSPECIFIED) {
    cout << "--> No lane, no gap." << endl;
    return -1.0;
  } else {
    cout << " " << endl;
  }

  double smallest_gap = REALLY_BIG_NUMBER;

  for(auto &otherCar: otherCars){

    double gap = (otherCar.s - myCar.s) * where;

    if (otherCar.lane == lane_type && gap > 0.0 && gap < smallest_gap) {
      smallest_gap = gap;
      this->current_front_v = otherCar.v;
      this->current_front_s = otherCar.s;
    }
  }

  return smallest_gap;
}
