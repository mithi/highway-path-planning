#ifndef TRAJECTORY_H_
#define TRAJECTORY_H_

#include <vector>
#include <iostream>
#include "helper.h"
#include "vehicle.h"
#include "jmt.h"
#include "trajectory.h"

class Trajectory {

  public:
    Trajectory(Vehicle& car, const BehaviorType behavior);
    State targetState_d;
    State targetState_s;
    JMT get_jmt_s() const;
    JMT get_jmt_d() const ;

  private:
    std::vector<JMT> jmtPair;

};

#endif //TRAJECTORY_H_
