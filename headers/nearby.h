#include "helper.h"
#include "vehicle.h"
#include <vector>

/* NearbyVehicleInfo - stores a vehicle object (vehicle),
 * how far away it is from us (gap) in road distance
 */
struct NearbyVehicleInfo {
  Vehicle car;
  double gap;
};

NearbyVehicleInfo get_nearest_front(
  const Vehicle &myCar, const std::vector<Vehicle>& cars, const LaneType lane_type){

  double smallest_gap = REALLY_BIG_NUMBER;
  NearbyVehicleInfo info;

  for(auto &otherCar: cars){

    double gap = otherCar.s - myCar.s;

    if (otherCar.lane == lane_type && gap > 0.0 && gap < smallest_gap) {

      info.car = otherCar;
      info.gap = gap;
      smallest_gap = gap;
    }
  }

  return info;
}

NearbyVehicleInfo get_nearest_back(
  const Vehicle &myCar, const std::vector<Vehicle>& cars, const LaneType lane_type){

  double smallest_gap = REALLY_BIG_NUMBER;
  NearbyVehicleInfo info;

  for(auto &otherCar: cars){

    double gap = myCar.s - otherCar.s;

    if (otherCar.lane == lane_type && gap > 0.0 && gap < smallest_gap) {

      info.car = otherCar;
      info.gap = gap;
      smallest_gap = gap;
    }
  }

  return info;
}
