#include "helper.h"
#include "vehicle.h"
#include <vector>

/* NearbyVehicleInfo - stores a vehicle object (vehicle),
 * how far away it is from us (gap) in road distance
 */
struct NearbyVehicleInfo {
  Vehicle car;
  double gap;
  bool is_empty;
};

/*
  NOTE: These two functions below could clearly use some refactoring and/or
        performance optimization IMHO
 */
NearbyVehicleInfo get_nearest_front(
  const Vehicle &myCar, const std::vector<Vehicle>& otherCars, const LaneType lane_type){

  double smallest_gap = REALLY_BIG_NUMBER;
  NearbyVehicleInfo info = {myCar, REALLY_BIG_NUMBER, true};

  if (lane_type == LaneType::NONE) {
    return info;
  }

  for(auto &otherCar: otherCars){

    double gap = otherCar.s - myCar.s;

    if (otherCar.lane == lane_type && gap > 0.0 && gap < smallest_gap) {

      info = { otherCar, gap, false };
      smallest_gap = gap;
    }
  }

  return info;
}

NearbyVehicleInfo get_nearest_back(
  const Vehicle &myCar, const std::vector<Vehicle>& otherCars, const LaneType lane_type){

  double smallest_gap = REALLY_BIG_NUMBER;
  NearbyVehicleInfo info = {myCar, REALLY_BIG_NUMBER, true};

  if (lane_type == LaneType::NONE) {
    return info;
  }

  for(auto &otherCar: otherCars){

    double gap = myCar.s - otherCar.s;

    if (otherCar.lane == lane_type && gap > 0.0 && gap < smallest_gap) {

      info = { otherCar, gap, false };
      smallest_gap = gap;
    }
  }

  return info;
}
