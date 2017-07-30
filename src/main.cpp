#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"

#include "pathconverter.h"
#include "vehicle.h"
#include "nearby.h"
#include "jmt.h"

using namespace std;

// For convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

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

int main() {

  uWS::Hub h;

  cout << "Loading map..." << endl;
  PathConverter pathConverter("../data/highway_map.csv", 6945.554);
  cout << "Map loaded..." << endl;

  bool just_starting = true;
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
          //* Get data from simulator
          //*********************************

          // j[1] is the data JSON object

        	// Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          vector<double> previous_path_x = j[1]["previous_path_x"];
          vector<double> previous_path_y = j[1]["previous_path_y"];
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          int n = previous_path_x.size();
          XYPoints XY_points = {previous_path_x, previous_path_y, n};

          //*********************************
          //* Update car objects
          //*********************************

          Vehicle myCar(66666);
          myCar.update_position(car_s, car_d);
          myCar.update_speed(car_speed, car_yaw);
          myCar.specify_adjacent_lanes();

          vector<Vehicle> otherCars;

          for (int i = 0; i < sensor_fusion.size(); i++) {

            int id = sensor_fusion[i][0];
            double s = sensor_fusion[i][5];
            double d = sensor_fusion[i][6];
            double vx = sensor_fusion[i][3];
            double vy = sensor_fusion[i][4];
            double v = sqrt(vx * vx + vy * vy);
            double heading = atan2(vy, vx);

             Vehicle car(id);
             car.update_position(s, d);
             car.update_speed(v, heading);
             otherCars.push_back(car);
          }

          if (just_starting) {
           //Our car hasn't moved yet. Let's move it!

            just_starting = false;

            State startState_s = {car_s, car_speed, 0.0};
            State startState_d = {car_d, 0.0, 0.0};

            double TIME_INCREMENT = 0.02;
            int NUMBER_OF_POINTS = 225;
            double TRAVERSE_TIME = 4.5;

            double GOAL_SPEED = 21.0;
            double GOAL_s = car_s + 0.5 * (car_speed + GOAL_SPEED) * TRAVERSE_TIME;

            State endState_s = {GOAL_s, GOAL_SPEED, 0.0};
            State endState_d = {myCar.convert_lane_to_d(myCar.lane), 0.0, 0.0};

            JMT jmt_s(startState_s, endState_s, TRAVERSE_TIME);
            JMT jmt_d(startState_d, endState_d, TRAVERSE_TIME);

            XY_points = pathConverter.make_path(jmt_s, jmt_d, TIME_INCREMENT, NUMBER_OF_POINTS);

          } else {
            cout << "finishing path :)" << endl;
          }

          json msgJson;

          // TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
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
