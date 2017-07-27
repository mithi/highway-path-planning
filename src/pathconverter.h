#ifndef PATHCONVERTER_H_
#define PATHCONVERTER_H_

#include <string>
#include <sstream>
#include <iostream>
#include <fstream>
#include <vector>
#include "spline.h"

/**
PathConverter contains the map of the highway. Use it to
convert Frenet Coordinates (s, d) to map coordinates (x, y) or to
convert Jerk Minimized Trajectories (JMT) to a path plan of (x, y) points on the map
**/

class PathConverter{

  public:
    PathConverter(std::string file_path, const double distance);
    /** Takes in a file containing map information and the distance of the map
        Each row of the file contains x y s dx dy values which are the waypoints.
        The x and y are the waypoint's map coordinate position.
        The s value is the distance along the road to get to that waypoint.
        The dx and dy values define the unit normal vector pointing outward of the highway loop.
    **/

    void save(std::string file_path, const double t, const int n);
    void save(std::string file_path, const double t, const int n, const double d);

    std::vector<double> convert_sd_to_xy(const double s, const double d);
    // void make_path(const& JMT jmt_s, const& JMT jmt_d, const double t, const int n);
    //std::vector<double>& get_x_path()
    //std::vector<double>& get_y_path()

  private:
   double distance;
   tk::spline x_spline;
   tk::spline y_spline;
   tk::spline dx_spline;
   tk::spline dy_spline;
};

#endif // PATHCONVERTER_H_
