#include "vehicle.h"

Vehicle::Vehicle(const int i){

  this->id = i;
  this->lane = LaneType::UNSPECIFIED;
}


void Vehicle::update_position(const double s, const double d){

  this->s = s;
  this->d = d;
  this->lane = this->convert_d_to_lane(this->d);
}


void Vehicle::update_speed(const double v, const double heading){

  this->v = v;
  this->heading = heading;
}


void Vehicle::update_state(const struct State& state){

  this->predicted_state = state;
  this->v = state.v;
  this->s = state.p;
}

void Vehicle::specify_adjacent_lanes(){

  if (this->lane == LaneType::LEFT) {

    this->lane_at_left = LaneType::NONE;
    this->lane_at_right = LaneType::MID;

  } else if (this->lane == LaneType::MID) {

    this->lane_at_left = LaneType::LEFT;
    this->lane_at_right = LaneType::RIGHT;

  } else if (this->lane == LaneType::RIGHT) {

    this->lane_at_left = LaneType::MID;
    this->lane_at_right = LaneType::NONE;

  } else {

    this->lane = LaneType::UNSPECIFIED;
    this->lane_at_left = LaneType::UNSPECIFIED;
    this->lane_at_right = LaneType::UNSPECIFIED;
  }
}


LaneType Vehicle::convert_d_to_lane(const double d){

  LaneType lane = LaneType::NONE;

  if (d > 0.0 && d < 4.0) {
    lane = LaneType::LEFT;
  } else if (d > 4.0 && d < 8.0) {
    lane = LaneType::MID;
  } else if (d > 8.0 && d < 12.0) {
    lane = LaneType::RIGHT;
  }

  return lane;
}

double Vehicle::convert_lane_to_d(const LaneType l){

  double d = -1;

  if (l == LaneType::LEFT) {
    d = 2.0;
  } else if (l == LaneType::MID) {
    d = 4.0;
  } else if (l == LaneType::RIGHT) {
    d = 6.0;
  }

  return d;
}
