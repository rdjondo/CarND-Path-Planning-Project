/*
 * trajectory.cpp
 *
 *  Created on: Oct 5, 2017
 *      Author: Raphael
 */
#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <mutex>
#include <thread>
#include <vector>
#include <atomic>

#include "spline.h"
#include "json.hpp"
#include "points.h"
#include "utils.h"
#include "jmt.h"
#include "optim_jmt.h"

using namespace std;

static void ideal_trajectory(std::vector<double> &map_waypoints_s,
    VectorPoints &map_waypoints, VectorPoints &previous_path,
    const int N_samples, double car_s, double car_speed,
    VectorPoints &next_vals) {
  double dist_inc = 0.4;
  static std::vector<double> next_s_vec;

  /* Define next initial position index values */
  double init_s = -1e9;
  int init_index = 0;

  if(next_s_vec.size()!=0){
    /* Linear search for initial s point to send back to simulator (motion smoothing)*/
    for(init_index=0 ;init_s < car_s && init_index<previous_path.size(); ++init_index){
      init_s = next_s_vec[init_index];
    }
    next_s_vec.clear();
  } else {
    init_s = car_s;
  }

  for (int i = 0; i < N_samples; i++) {
    double next_s = init_s + (i + 1) * dist_inc;
    next_s_vec.push_back(next_s);
    double next_d = 6;
    Point pt = getXY(next_s, next_d, map_waypoints_s, map_waypoints);
    next_vals.push_back(pt);
  }
}

static void my_trajectory(std::vector<double> &map_waypoints_s,
    VectorPoints &map_waypoints, VectorPoints &previous_path,
    const int N_samples, double car_s, double car_speed,
    VectorPoints &next_vals) {
  // DONE: smoothen road path between the waypoints using Spline

  // TODO: Calculate and control SDC vehicle speed and lane position
  // Use a Jerk minimizing function to control the vehicle speed (use the two past points for continuity)
  // Let's compute the Jerk minimization in the Frenet referential

  const double delta_t = 0.02;
  static vector<double> coeffs_old = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
  static std::vector<double> next_s_vec;

  const int previous_path_size = previous_path.size();
  double sk;
  double sk_dot;
  double sk_double_dot = 0.0;

  double sT_dot = 10.5;
  double sT_double_dot = 0.0;


  /* Define next initial position index values */
  double init_s = -1e9;
  int init_index;


  if (next_s_vec.size() > 1) {
    /* Linear search for initial s point to send back to simulator (motion smoothing)*/
    for (init_index = 0; init_s < car_s && init_index < previous_path.size();
        ++init_index) {
      init_s = next_s_vec[init_index];
    }
    next_s_vec.clear();

  } else {
    init_index = 0;
    init_s = car_s;
    sk_dot = car_speed / 2.23694; /* Comvert Mi/hr to m/s */
  }

  /* New time this is an ugly hyper-parameter optimization.
   * 13 is a "magic" number that compensates for the computation lag */
  double delay_t = (init_index+13) * delta_t;

  /* Display estimated telemetry from old plan */
  sk = init_s;
  vector<double> speed_poly = polyder(coeffs_old);
  sk_dot = polyval(speed_poly, delay_t);
  vector<double> acc_poly = polyder(speed_poly);
  sk_double_dot = polyval(acc_poly, delay_t);

  cout << "last_s:" << init_s << "   last_t:" << delay_t << "\n";
  cout << "len(previous_path_x):" << previous_path_size << "\n";
  cout << "car_s:" << car_s << "  estim car_s:" << polyval(coeffs_old, delay_t)
      << "\n";
  cout << "car_speed:" << car_speed << "  estim car_speed:"
      << sk_dot * 2.23694 * 2 << "\n";
  cout << "init_index:" << init_index << "  estim sT_dot:" << sT_dot << "\n";


  double sT_optimised = 0.0;
  double T_optimised = 0.0;

  vector<double> coeffs = optim_jmt(sk, sk_dot, sk_double_dot, sT_dot,
      sT_double_dot, sT_optimised, T_optimised);

  coeffs_old = coeffs;

  cout << "coeffs = [" << coeffs[0] << ", " << coeffs[1] << ", " << coeffs[2]
      << ", " << coeffs[3] << ", " << coeffs[4] << ", " << coeffs[5] << "] "
      << endl;

  next_vals.clear();

  for (int i = 0; i < N_samples; ++i, i++) {
    double next_s = polyval(coeffs, (i) * delta_t);
    next_s_vec.push_back(next_s);

    double next_d = 6;
    Point pveh = getXY(next_s, next_d, map_waypoints_s, map_waypoints);
    //vector<double> xy = mapCoordToVehicleCoordinates(car_x, car_y, car_yaw, xy_veh[0], xy_veh[1]);
    //xy = mapPtVehCoordToMapCoordinates(car_x, car_y, car_yaw, xy[0], xy[1]);
    next_vals.push_back(pveh);
  }
}

void trajectory(std::vector<double> &map_waypoints_s,
    VectorPoints &map_waypoints, VectorPoints &previous_path,
    const int N_samples, double car_s, double car_speed,
    VectorPoints &next_vals) {
  my_trajectory(map_waypoints_s, map_waypoints, previous_path, N_samples,
      car_s, car_speed, next_vals);

}

