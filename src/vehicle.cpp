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


void Vehicle::update_speed(const double v){

  this->v = v;
}


void Vehicle::update_save_states(const State& state_s, const State& state_d){

  this->saved_state_s = state_s;
  this->saved_state_d = state_d;
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

LaneType Vehicle::convert_d_to_lane(){
  return this->convert_d_to_lane(this->d);
}

double Vehicle::convert_lane_to_d(const LaneType l){

  double d = -1;

  if (l == LaneType::LEFT) {
    d = 2.2;
  } else if (l == LaneType::MID) {
    d = 6.0;
  } else if (l == LaneType::RIGHT) {
    d = 9.8;
  }

  return d;
}

double Vehicle::convert_lane_to_d(){

  return this->convert_lane_to_d(this->lane);
}
