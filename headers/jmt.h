#ifndef JMT_H_
#define JMT_H_

#include <iostream>
#include "../src/Eigen-3.3/Eigen/Core"
#include "../src/Eigen-3.3/Eigen/QR"
#include "../src/Eigen-3.3/Eigen/Dense"
#include "helper.h"

class JMT {

  public:
    Eigen::VectorXd c;
    JMT(State start, State end, const double t);
    double get(const double t);
};

#endif // JMT_H_
