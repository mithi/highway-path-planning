#include <vector>
#include "helper.h"
#include "vehicle.h"

/* Get gap between nearest vehicle from either our front or back
   at a given given lane
  NOTE: where: FROM_FRONT = 1.0, FROM_BACK = -1
 */
double get_gap(
  const Vehicle &myCar, const std::vector<Vehicle>& otherCars, const LaneType lane_type, double where){

  double smallest_gap = REALLY_BIG_NUMBER;

  for(auto &otherCar: otherCars){

    double gap = (otherCar.s - myCar.s) * where;

    if (otherCar.lane == lane_type && gap > 0.0 && gap < smallest_gap) {
      smallest_gap = gap;
    }
  }

  return smallest_gap;
}


/* NearbyVehicleInfo - stores a vehicle object (vehicle),
 * how far away it is from us (gap) in road distance
 */
struct NearbyVehicleInfo {
  Vehicle car;
  double gap;
  bool is_empty;
};

/*
  NOTE: - These two functions below could clearly use some refactoring and/or
          performance optimization IMHO.
        - Also currently, the computation does not take into account that this
          is a highway loop. We should probably use modulo for this
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
