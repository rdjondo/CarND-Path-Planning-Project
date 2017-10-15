#include <cmath>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <atomic>
#include <mutex>
#include <thread>
#include <vector>
#include <functional> // For ref wrapper
#include "json.hpp"
#include "spline.h"
#include "points.h"
#include "utils.h"
#include "trajectory.h"

using namespace std;

// for convenience
using json = nlohmann::json;

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  VectorPoints map_waypoints;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  loadMap(map_waypoints, map_waypoints_s, map_waypoints_dx, map_waypoints_dy);
  RoadGeometry road(map_waypoints, map_waypoints_s);

  int max_loops = 10000;
  VectorPoints next_vals;
  next_vals.clear();
  DoubleBuffer<VectorPoints> log;

  std::thread logging_thread(logWaypoints, max_loops, std::cref(next_vals), std::ref(log));

  h.onMessage([&road, &map_waypoints_dx, &map_waypoints_dy,
  &next_vals, & log](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
      uWS::OpCode opCode)
      {
        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
        //auto sdata = string(data).substr(0, length);
        //cout << sdata << endl;
        if (length && length > 2 && data[0] == '4' && data[1] == '2')
        {

          auto s = hasData(data);

          if (s != "")
          {
            auto j = json::parse(s);

            string event = j[0].get<string>();

            if (event == "telemetry")
            {
              // j[1] is the data JSON object

              // Main car's localization Data
              double car_x = j[1]["x"];
              double car_y = j[1]["y"];
              double car_s = j[1]["s"];
              double car_d = j[1]["d"];
              double car_yaw = j[1]["yaw"];
              double car_speed = j[1]["speed"];

              // Previous path data given to the Planner
              json previous_path_x_json = j[1]["previous_path_x"];
              json previous_path_y_json = j[1]["previous_path_y"];
              // Previous path's end s and d values
              double end_path_s = j[1]["end_path_s"];
              double end_path_d = j[1]["end_path_d"];

              // Sensor Fusion Data, a list of all other cars on the same side of the road.
              vector<vector<double>> sensor_fusion = j[1]["sensor_fusion"];

              json msgJson;

              const int N_samples = 170;//(int) T_optimised/delta_t;
              next_vals.reserve(N_samples);
              next_vals.clear();

              // TODO: Calculate time and distance to collision against other vehicles in Frenet

              // TODO: Design cost function

              // TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
              VectorPoints previous_path;
              previous_path.setPoints(previous_path_x_json, previous_path_y_json);

              trajectory(road, previous_path, N_samples, car_s, car_d,
                  car_speed,car_yaw, next_vals);

              msgJson["next_x"] = next_vals.getVectorX();
              msgJson["next_y"] = next_vals.getVectorY();
              log.update(next_vals);

              auto msg = "42[\"control\"," + msgJson.dump() + "]";

              ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
              // std::cout<< "DOWN:" << j<<std::endl;
              // std::cout<< "UP:" << msg<<std::endl;
              this_thread::sleep_for(chrono::milliseconds(500));
            }
          }
          else
          {
            // Manual driving
            std::string msg = "42[\"manual\",{}]";
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          }
        }
      });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
      size_t, size_t)
  {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req)
  {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
      char *message, size_t length)
  {
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
  logging_thread.join();

}
