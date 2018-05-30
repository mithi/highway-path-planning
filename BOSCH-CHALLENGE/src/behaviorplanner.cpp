#include "behaviorplanner.h"

using namespace std;

BehaviorPlanner::BehaviorPlanner() {}

BehaviorType BehaviorPlanner::update(Vehicle& car, std::vector<Vehicle>& otherCars) {

  const double KEEPGAP = 30.0;

  car.front_gap = this->get_effective_gap(car, otherCars, car.lane, FROM_FRONT);
  car.front_v = this->current_v;

  if (car.front_gap > KEEPGAP) {
    return BehaviorType::KEEPLANE;
  }

  if (car.front_gap < 5.0) { // should emergency break
    return BehaviorType::KEEPLANE;
  }

  this->update_decision_factors(car, otherCars);

  if (car.lane == LaneType::MID){
    return this->midlane_decide(car);
  }

  return this->nonmidlane_decide(car);
}

BehaviorType BehaviorPlanner::midlane_decide(Vehicle &car) {

  const double left_cost = 1.5 * this->get_cost(this->backleft_gap, this->frontleft_gap);
  const double right_cost = 1.5 * this->get_cost(this->backright_gap, this->frontright_gap);
  const double CUTOFF = this->get_cost(REALLY_BIG_NUMBER, car.front_gap);

  cout << "costs: TURNLEFT | STRAIGHT | TURNRIGHT" << endl;
  cout << left_cost << " ---- " << CUTOFF << " ----" << right_cost << endl;

  if (CUTOFF < left_cost && CUTOFF < right_cost) {
    return BehaviorType::KEEPLANE;
  }

  return left_cost < right_cost ? BehaviorType::TURNLEFT : BehaviorType::TURNRIGHT;
}

double BehaviorPlanner::get_cost(const double back, const double front) {
  return (1.0 / front + 0.5 / back);
}

BehaviorType BehaviorPlanner::nonmidlane_decide(Vehicle &car) {

  const double CUTOFF = this->get_cost(DUMMY_BACK, car.front_gap);
  double cost = CUTOFF;

  if (car.lane_at_left == LaneType::MID) {

    cost = this->get_cost(this->backleft_gap, this->frontleft_gap);
    return cost < CUTOFF ? BehaviorType::TURNLEFT : BehaviorType::KEEPLANE;

  } else {

    cost = this->get_cost(this->backright_gap, this->frontright_gap);
    return cost < CUTOFF ? BehaviorType::TURNRIGHT : BehaviorType::KEEPLANE;
  }
}

void BehaviorPlanner::update_decision_factors(Vehicle& car, std::vector<Vehicle>& otherCars) {

  this->backleft_gap = this->get_effective_gap(car, otherCars, car.lane_at_left, FROM_BACK);
  this->frontleft_gap = this->get_effective_gap(car, otherCars, car.lane_at_left, FROM_FRONT);
  this->backright_gap = this->get_effective_gap(car, otherCars, car.lane_at_right, FROM_BACK);
  this->frontright_gap = this->get_effective_gap(car, otherCars, car.lane_at_right, FROM_FRONT);

/*
  cout << "-" << endl;
  cout << "LEFT:     f: " << this->frontleft_gap << endl;
  cout << "          b: " << this->backleft_gap << endl;
  cout << "-" << endl;
  cout << "RIGHT     f: " << this->frontright_gap << endl;
  cout << "          b: " << this->backright_gap << endl;
  cout << "-" << endl;
  cout << "STRAIGHT  f: " << car.front_gap  << endl;
  cout << "          v: " << car.front_v << endl;
*/
}

double BehaviorPlanner::get_gap(
  const Vehicle &car, const std::vector<Vehicle>& otherCars, const LaneType lane_type, const double where) {
  // get gap and nearest neighbor information

  if (lane_type == LaneType::NONE || lane_type == LaneType::UNSPECIFIED) {
    this->current_v = 0.0;
    return 0.000001;
  }

  double smallest_gap = REALLY_BIG_NUMBER;

  for (auto &otherCar: otherCars) {

    double gap = (otherCar.s - car.s) * where;

    if (otherCar.lane == lane_type && gap > 0.0 && gap < smallest_gap) {
      smallest_gap = gap;
      this->current_v = otherCar.v;
    }
  }

  return smallest_gap;
}

double BehaviorPlanner::get_effective_gap(
  const Vehicle &car, const std::vector<Vehicle>& otherCars, const LaneType lane_type, const double where) {

  const double LATENCY = 0.75;
  const double BUFFER = 5.0;

  if (lane_type == LaneType::NONE || lane_type == LaneType::UNSPECIFIED) {
    this->current_v = 0.0;
    return 0.000001;
  }

  double smallest_gap = REALLY_BIG_NUMBER;

  // get gap and nearest neighbor information

  for (auto &otherCar: otherCars) {

    const double other_s = otherCar.s + LATENCY * otherCar.v;
    const double my_s = car.s + LATENCY * car.saved_state_s.v;
    const double gap = (other_s - my_s) * where;

    if (otherCar.lane == lane_type && gap > 0.0 && gap < smallest_gap) {
      smallest_gap = gap;
      this->current_v = otherCar.v;
    }
  }

  smallest_gap = max(0.00001, smallest_gap - BUFFER);
  return smallest_gap;
}
