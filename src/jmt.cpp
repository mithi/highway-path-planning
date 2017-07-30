#include "jmt.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

JMT::JMT(const State& start, const State& end, const double t) {

  MatrixXd A = MatrixXd(3,3);
  VectorXd b = VectorXd(3);
  VectorXd x = VectorXd(3);
  this->c = VectorXd(6);

  const double t2 = t * t;
  const double t3 = t * t2;
  const double t4 = t * t3;
  const double t5 = t * t4;

    A <<   t3,     t4,    t5,
         3*t2,   4*t3,  5*t4,
         6*t ,  12*t2, 20*t3;

    b << end.p - (start.p + start.v * t + 0.5 * start.a * t2),
         end.v - (start.v + start.a * t),
         end.a -  start.a;

    x = A.inverse() * b;

    this->c << start.p,
               start.v,
               start.a / 2.0,
               x[0],
               x[1],
               x[2];

    //std::cout << this->c << std::endl;
}

double JMT::get(const double t) const{

  const double t2 = t * t;
  const double t3 = t * t2;
  const double t4 = t * t3;
  const double t5 = t * t4;

  Eigen::VectorXd T = VectorXd(6);
  T << 1.0, t, t2, t3, t4, t5;

  return T.transpose() * this->c;
}
