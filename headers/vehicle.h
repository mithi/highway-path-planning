#ifndef VEHICLE_H_
#define VEHICLE_H_

#include <iostream>
#include <vector>
#include <math.h>
#include "helper.h"

class Vehicle {

  public:

    int id;
    double s;
    double d;
    double v;
    double front_gap;
    double front_v;
    double front_s;

    State saved_state_s;
    State saved_state_d;

    LaneType lane;
    LaneType lane_at_right;
    LaneType lane_at_left;

    Vehicle(const int i);

    void update_position(const double s, const double d);
    void update_speed(const double v);
    void update_save_states(const State& state_s, const State& state_d);
    void specify_adjacent_lanes();

    LaneType convert_d_to_lane(const double d);
    LaneType convert_d_to_lane();
    double convert_lane_to_d(const LaneType l);
    double convert_lane_to_d();
    double get_target_d(const BehaviorType b);
};

#endif //VEHICLE_H_
