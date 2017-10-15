/*
 * test.cpp
 *
 *  Created on: Oct 5, 2017
 *      Author: Raphael
 */

#include <cmath>
#include <chrono>
#include <fstream>
#include <sstream>
#include <iostream>
#include <mutex>
#include <thread>
#include <random>
#include <vector>
#include <atomic>
#include <functional> // For ref wrapper

#include "json.hpp"
#include "spline.h"
#include "points.h"
#include "utils.h"
#include "trajectory.h"

using namespace std;


int main() {
  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  VectorPoints map_waypoints;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  loadMap(map_waypoints, map_waypoints_s, map_waypoints_dx, map_waypoints_dy);
  RoadGeometry road(map_waypoints, map_waypoints_s);

  DoubleBuffer<VectorPoints> log;

  const int N_samples = 800;


  double dist_inc = 10.0;
  cout<<"road.getMaxS()="<<road.getMaxS()<<endl;

  ofstream logfile;
  logfile.open("trajectory_calc.csv");

  logfile << "s_req,s,x,y\n";

  for (int i = 0; i < N_samples; i++) {
    double s_req = i * dist_inc;
    Point pt = road.getXY(s_req, 6);
    double s = s_req;
    if (s > road.getMaxS()) {
      s = fmod(s_req, road.getMaxS());
    }
    logfile << s_req << "," << s << "," << pt.x << "," << pt.y << "\n";
  }
  logfile << endl;
  logfile.close();

  return 0;
}
