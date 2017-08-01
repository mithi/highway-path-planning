#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include <math.h>

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"

#include "pathconverter.h"
#include "vehicle.h"
#include "jmt.h"
#include "behaviorplanner.h"
#include "trajectory.h"

using namespace std;

// For convenience
using json = nlohmann::json;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned, else the empty string "" will be returned.
string hasData(string s) {

  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");

  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }

  return "";
}


void print_lane(LaneType lane){

  if (lane == LaneType::LEFT) {
    cout << "LEFT       ";
  } else if (lane == LaneType::MID) {
    cout << "MID        ";
  } else if (lane == LaneType::RIGHT) {
    cout << "RIGHT      ";
  } else if (lane == LaneType::NONE) {
    cout << "NONE       ";
  } else if (lane == LaneType::UNSPECIFIED) {
    cout << "UNSPECIFIED";
  }
}


XYPoints start_engine(Vehicle& car, PathConverter& pathConverter){

  const int n = 225;
  const double t = n * TIME_INCREMENT;
  const double target_speed = 20.0;
  const double target_s = car.s + 40.0;

  const State startState_s = {car.s, car.v, 0.0};
  const State startState_d = {car.d, 0.0, 0.0};

  const State endState_s = {target_s, target_speed, 0.0};
  const State endState_d = {car.convert_lane_to_d(), 0.0, 0.0};

  JMT jmt_s(startState_s, endState_s, t);
  JMT jmt_d(startState_d, endState_d, t);

  car.update_save_states(endState_s, endState_d);

  return pathConverter.make_path(jmt_s, jmt_d, TIME_INCREMENT, n);
}


int main() {

  uWS::Hub h;

  cout << "Program Started." << endl;
  bool just_starting = true;

  cout << "Loading map..." << endl;
  PathConverter pathConverter("../data/highway_map.csv", TRACK_DISTANCE);
  cout << "Map loaded..." << endl;


  h.onMessage([&pathConverter, &just_starting](

    uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {

    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;

    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {

          //*********************************
          //* Get relevant data from simulator
          //*********************************

          // j[1] is the data JSON object
          
          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];

          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          vector<double> previous_path_x = j[1]["previous_path_x"];
          vector<double> previous_path_y = j[1]["previous_path_y"];
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          auto sensor_fusion = j[1]["sensor_fusion"];

          //*********************************
          //* Update car object
          //*********************************

          Vehicle myCar(666); // I just like 666
          myCar.update_position(car_s, car_d);
          myCar.update_speed(car_speed);
          myCar.specify_adjacent_lanes();

          //*********************************
          //* Generate the XY_points which will be sent to the simulator
          //*********************************

          // Our default is to just give back our previous plan
          int n = previous_path_x.size();
          XYPoints XY_points = {previous_path_x, previous_path_y, n};

          if (just_starting) {

            //Our car hasn't moved yet. Let's move it!
            cout << "Starting engine..." << endl;
            XY_points = start_engine(myCar, pathConverter);
            just_starting = false;
            cout << "Engine started..." << endl;

          } else if (n < PATH_SIZE_CUTOFF) {

            // Our previous plan is about to run out, so append to it
            // Make a list of all relevant information about other cars
            vector<Vehicle> otherCars;

            for (int i = 0; i < sensor_fusion.size(); i++) {

              int id = sensor_fusion[i][0];
              double s = sensor_fusion[i][5];
              double d = sensor_fusion[i][6];
              double vx = sensor_fusion[i][3];
              double vy = sensor_fusion[i][4];

              Vehicle car(id);
              car.update_position(s, d);
              car.update_speed(sqrt(vx * vx + vy * vy));
              otherCars.emplace_back(car);
            }

            // Print for debugging
            cout << "---------------------------------" << endl;
            cout << "STATE: s, d --- x, y --- v:" << endl;
            cout << car_s << " , "
                 << car_d << " --- "
                 << car_x << " , "
                 << car_y  << " --- "
                 << car_speed << ":" << endl;

            cout << "---------------------------------" << endl;
            cout << "our left:  our lane:   our right:" << endl;
            print_lane(myCar.lane_at_left);
            print_lane(myCar.lane);
            print_lane(myCar.lane_at_right);
            cout << endl;

            cout << "---------------------------------" << endl;

            // Decide whether to turn left, turn right or keeplane based on data at hand
            // NOTE: BehaviorPlanner updates our car's current leading/front vehicle's speed and gap

            BehaviorPlanner planner;
            BehaviorType behavior = planner.update(myCar, otherCars);

            // This trajectory generates target states and then from this
            // generates a jerk minimized trajectory fucntion
            // given the impending state and suggested behavior
            Trajectory trajectory(myCar, behavior);

            // Update saved state of our car (THIS IS IMPORTANT) with the latest
            // generated target states, this is to be used as the starting state
            // when generating a trajectory next time
            myCar.update_save_states(trajectory.targetState_s, trajectory.targetState_d);

            // convert this trajectory in the s-d frame to to discrete XY points
            // the simulator can understand
            XYPoints NextXY_points = pathConverter.make_path(
              trajectory.get_jmt_s(), trajectory.get_jmt_d(), TIME_INCREMENT, NUMBER_OF_POINTS);

            NextXY_points.n = NUMBER_OF_POINTS;

            // Append these generated points to the old points
            XY_points.xs.insert(
              XY_points.xs.end(), NextXY_points.xs.begin(), NextXY_points.xs.end());

            XY_points.ys.insert(
              XY_points.ys.end(), NextXY_points.ys.begin(), NextXY_points.ys.end());

            XY_points.n = XY_points.xs.size();
          }

          //*********************************
          //* Send updated path plan to simulator
          //*********************************

          json msgJson;

          msgJson["next_x"] = XY_points.xs;
          msgJson["next_y"] = XY_points.ys;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          //this_thread::sleep_for(chrono::milliseconds(1000));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }

      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the program doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {

    const std::string s = "<h1>Hello world!</h1>";

    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      res->end(nullptr, 0); // i guess this should be done more gracefully?
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;

  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }

  h.run();
}
