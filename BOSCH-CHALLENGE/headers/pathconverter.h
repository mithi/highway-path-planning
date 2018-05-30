#ifndef PATHCONVERTER_H_
#define PATHCONVERTER_H_

#include <string>
#include <sstream>
#include <iostream>
#include <fstream>
#include <math.h>
#include <vector>

#include "spline.h"
#include "helper.h"
#include "jmt.h"

/**
PathConverter contains the map of the highway. Use it to
convert Frenet Coordinates (s, d) to map coordinates (x, y) or to
convert Jerk Minimized Trajectories (JMT) to a path plan (XYPath) of (x, y) points on the map
**/

/**
map file convention stored at file path:
Each row of the file contains x y s dx dy values which are the waypoints.
The x and y are the waypoint's map coordinate position.
The s value is the distance along the road to get to that waypoint.
The dx and dy values define the unit normal vector pointing outward of the highway loop.
 **/

class PathConverter{

  public:
    PathConverter(std::string file_path, const double distance);
    /** Takes in: file_path - a file path containing map information
                  distance - the total road distance of the map
    **/

    std::vector<double> convert_sd_to_xy(const double s, const double d) const;
    /**
      Takes in (s, d) coordinates in the frenet frame (which is along the loop of the road)
      Returns a vector (x, y) which are cartesian coordinates in the fixed map frame
     **/

    XYPoints make_path(JMT jmt_s, JMT jmt_d, const double t, const int n) const;

    void save(std::string file_path, const double t, const int n) const;
    /** Takes in: file_path - path where the waypoints will be saved
                  t - distance between way points
                  n - number of way points
     **/

    void save(std::string file_path, const double t, const int n, const double d) const;
    /** Takes in: file_path - path where the waypoints will be saved
                  t - distance between way points
                  n - number of waypoints
                  d - distance of the waypoint outward the highway loop
     **/


  private:
   double distance;
   tk::spline x_spline;
   tk::spline y_spline;
   tk::spline dx_spline;
   tk::spline dy_spline;
};

#endif // PATHCONVERTER_H_
