#ifndef BEHAVIORPLANNER_H_
#define BEHAVIORPLANNER_H_

#include <vector>
#include <iostream>

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
  private:
    double current_front_v;
    double current_front_s;
    
    double get_cost(const double front_gap, const double back_gap, const LaneType lane) const;
    double get_cost(const double gap) const;
};

#endif //BEHAVIORPLANNER_H_
