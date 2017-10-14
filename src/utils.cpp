/*
 * utils.cpp
 *
 *  Created on: 30 Sep 2017
 *      Author: rdjondo
 */

// Load up map values for waypoint's x,y,s and d normalized normal vectors
#include <cmath>
#include <fstream>
#include <sstream>
#include <iostream>
#include <vector>
#include <atomic>
#include <mutex>
#include <thread>
#include <chrono>
#include <string>
#include "json.hpp"
#include "spline.h"
#include "points.h"
#include "utils.h"

using namespace std;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
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

void loadMap(VectorPoints &map_waypoints, vector<double> &map_waypoints_s,
    vector<double> &map_waypoints_dx, vector<double> &map_waypoints_dy) {

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    Point pt = { x, y };
    map_waypoints.push_back(pt);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }
}

void logWaypoints(int max_loops, const VectorPoints & next_vals, DoubleBuffer<VectorPoints> & log) {

  ofstream logfile;
  logfile.open("trajectory_log.csv");

  logfile << "seq,x,y\n";

  for (int i = 0; i < max_loops; ++i) {

    log.get().printCsv(logfile, i);

    this_thread::sleep_for(chrono::milliseconds(200));
  }

  logfile<<endl;

  logfile.close();

  cout<<endl<<"Finished Logging!!!!"<<endl;

}
