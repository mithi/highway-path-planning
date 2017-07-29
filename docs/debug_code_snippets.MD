```c

  cout << "Loading map..." << endl;
  PathConverter pathConverter("../data/highway_map.csv", 6945.554);
  // Uncomment should you wish to save each lane to a different csv file
  //pathConverter.save("../data/finegrained_map.csv", 1.0, 6945);
  //pathConverter.save("../data/leftlane_map.csv", 1.0, 6945, 2.0);
  //pathConverter.save("../data/midlane_map.csv", 1.0, 6945, 6.0);
  //pathConverter.save("../data/rightlane_map.csv", 1.0, 6945, 10.0);
  //pathConverter.save("../data/farrightlane_map.csv", 1.0, 6945, 50.0);
  cout << "Map loaded..." << endl;
  
          //*********************************
          //* DEBUG CAR OBJECTS
          //*********************************
          cout << "my car: " << myCar.id << endl;
          cout << "--position: " << myCar.s << " , " << myCar.d << endl;
          cout << "--speed: " << myCar.v << " , " << myCar.heading << endl;
          cout << "--lane:";

          if (myCar.lane == LaneType::LEFT) {
            cout << "left" << endl;
          }
          if (myCar.lane == LaneType::MID) {
            cout << "mid" << endl;
          }
          if (myCar.lane == LaneType::RIGHT) {
            cout << "right" << endl;
          }

          for(auto &car: otherCars) {

            cout << "car id: " << car.id << endl;
            cout << "--position: " << car.s << " , " << car.d << endl;
            cout << "--speed: " << car.v << " , " << car.heading << endl;
          }

          //*********************************
          //* DEBUG NEARBY FUNCTIONS
          //*********************************

          NearbyVehicleInfo frontleft = get_nearest_front(myCar, otherCars, myCar.lane_at_left);
          NearbyVehicleInfo frontright = get_nearest_front(myCar, otherCars, myCar.lane_at_right);
          NearbyVehicleInfo front = get_nearest_front(myCar, otherCars, myCar.lane);

          NearbyVehicleInfo backleft = get_nearest_back(myCar, otherCars, myCar.lane_at_left);
          NearbyVehicleInfo backright = get_nearest_back(myCar, otherCars, myCar.lane_at_right);
          NearbyVehicleInfo back = get_nearest_back(myCar, otherCars, myCar.lane);

          cout << "has in front:" << !front.is_empty
               << " ---> id: " << front.car.id << " gap: " << front.gap << endl;

          cout << "has in back: " << !back.is_empty
               << " ---> id: " << back.car.id << " gap: " << back.gap << endl;

          cout << "has in frontleft: " << !frontleft.is_empty
               << " ---> id: " << frontleft.car.id << " gap: " << frontleft.gap << endl;

          cout << "has in backleft: " << !backleft.is_empty
               << " ---> id: " << backleft.car.id << " gap: " << backleft.gap << endl;

          cout << "has in frontright: " << !frontright.is_empty
               << " ---> id: " << frontright.car.id << " gap: " << frontright.gap << endl;

          cout << "has in backright: " << !backright.is_empty
               << " ---> id: " << backright.car.id << " gap: " << backright.gap << endl;

          //*********************************
          //* DEBUG JMT OBJECT
          //*********************************
          cout << "JMT1..." << endl;
          JMT jmt1({0.0, 10.0, 0.0}, {10.0, 10.0, 0.0}, 1.0);
          cout << "->" << "0.0, 10.0, 0.0, 0.0, 0.0, 0.0" << endl;

          cout << "JMT2..." << endl;
          JMT jmt2({0.0, 10.0, 0.0}, {20.0, 15.0, 20.0}, 2.0);
          cout << "->" << "0.0, 10.0, 0.0, 0.0, -0.625, 0.3125" << endl;

          cout << "JMT3..." << endl;
          JMT jmt3({5.0, 10.0, 2.0}, {-30.0, -20.0, -4.0}, 5.0);
          cout << "->" << "5.0, 10.0, 1.0, -3.0, 0.64, -0.0432" << endl;

```
