#ifndef BEHAVIORPLANNER_H_
#define BEHAVIORPLANNER_H_

#include <vector>
#include <iostream>
#include <algorithm>
#include "helper.h"
#include "vehicle.h"

class BehaviorPlanner{

  public:
    BehaviorPlanner();
    BehaviorType update(Vehicle& myCar, std::vector<Vehicle>& otherCars);
    double get_gap(const Vehicle &myCar,
                   const std::vector<Vehicle>& otherCars,
                   const LaneType lane_type,
                   const double where);
    double get_effective_gap(const Vehicle &myCar,
                   const std::vector<Vehicle>& otherCars,
                   const LaneType lane_type,
                   const double where);

  private:
    double current_v;
    double backleft_gap;
    double backright_gap;
    double frontleft_gap;
    double frontright_gap;

    double get_cost(const double back, const double front);
    void update_decision_factors(Vehicle& car, std::vector<Vehicle>& otherCars);
    BehaviorType midlane_decide(Vehicle &car);
    BehaviorType nonmidlane_decide(Vehicle& car);
};

#endif //BEHAVIORPLANNER_H_
