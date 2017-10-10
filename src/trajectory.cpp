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
#include "trajectory.h"
#include "optim_jmt.h"

using namespace std;

static void ideal_trajectory(std::vector<double> &map_waypoints_s,
    VectorPoints &map_waypoints, VectorPoints &previous_path,
    const int N_samples, double car_s, double car_d, double car_speed,
    VectorPoints &next_vals) {
  double dist_inc = 0.4;
  static std::vector<double> next_s_vec;

  /* Define next initial position index values */
  double init_s = -1e9;
  size_t init_index = 0;

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
    const int N_samples, double car_s, double car_d, double car_speed,
    VectorPoints &next_val_xy) {
  // DONE: smoothen road path between the waypoints using Spline

  // DONE: Calculate and control SDC vehicle speed and lane position
  // Use a Jerk minimizing function to control the vehicle speed (use the two past points for continuity)
  // Let's compute the Jerk minimization in the Frenet referential

  // TODO : Fix loop closure discontinuity

  const double delta_t = 0.02;
  static vector<double> coeff_s_old = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
  static vector<double> coeff_d_old = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
  static std::vector<double> next_s_vec;
  static std::vector<double> next_d_vec;

  double sk;
  double sk_dot;
  double sk_double_dot = 0.0;

  double sT_dot = 10.5;
  double sT_double_dot = 0.0;

  double dk;
  double dk_dot;
  double dk_double_dot = 0.0;

  double dT_dot = 0.0;
  double dT_double_dot = 0.0;


  /* Define next initial position index values */
  double init_s = -1e9;
  double init_d = -1e9;
  size_t init_index;


  if (next_s_vec.size() > 1) {
    /* Linear search for initial s point to send back to simulator (motion smoothing)*/
    for (init_index = 0; init_s < car_s && init_index < previous_path.size();
        ++init_index) {
      init_s = next_s_vec[init_index];
      init_d = next_d_vec[init_index];
    }
    next_s_vec.clear();
    next_d_vec.clear();

  } else {
    init_index = 0;
    init_s = car_s;
    init_d = 6;
    sk_dot = car_speed / 2.23694; /* Comvert Mi/hr to m/s */
  }

  /* New time this is an ugly hyper-parameter optimization.
   * 13 is a "magic" number that compensates for the computation lag */
  double delay_t = (init_index+13) * delta_t;

  /* Display estimated telemetry from old plan */
  sk = init_s;
  vector<double> speed_poly_s = polyder(coeff_s_old);
  sk_dot = polyval(speed_poly_s, delay_t);
  vector<double> acc_poly_s = polyder(speed_poly_s);
  sk_double_dot = polyval(acc_poly_s, delay_t);


  //TODO: Only allow for lane change if speed s_dot greater than minimal speed.
  double dT = 6.0;
  if(((int)floor(car_s/100))%4==0){
    dT = 2.0;
  }

  /* Display estimated telemetry from old plan */
  dk = init_d;
  vector<double> speed_poly_d = polyder(coeff_d_old);
  dk_dot = polyval(speed_poly_d, delay_t);
  vector<double> acc_poly_d = polyder(speed_poly_d);
  dk_double_dot = polyval(acc_poly_d, delay_t);


  double virtual_acceleration = 1.0; /* virtual acceleration in m/s^2 */

  vector<double> coeff_s = optim_jmt_affine(sk, sk_dot, sk_double_dot, sT_dot,
      sT_double_dot, virtual_acceleration);


  double T = 8.0;
  vector<double> coeff_d = optim_jmt_quadratic( dk,  dk_dot,  dk_double_dot,
      dT,  dT_dot,  dT_double_dot, T);

  coeff_s_old = coeff_s;
  coeff_d_old = coeff_d;

  cout << "coeffs = [" << coeff_s[0] << ", " << coeff_s[1] << ", " << coeff_s[2]
      << ", " << coeff_s[3] << ", " << coeff_s[4] << ", " << coeff_s[5] << "] "
      << endl;

  next_val_xy.clear();

  //double next_d = 6;

  for (int i = 0; i < N_samples; ++i, i++) {
    double next_s = polyval(coeff_s, (i) * delta_t);
    double next_d = polyval(coeff_d, (i) * delta_t);
    next_s_vec.push_back(next_s);
    next_d_vec.push_back(next_d);

    Point pveh = getXY(next_s, next_d, map_waypoints_s, map_waypoints);
    //vector<double> xy = mapCoordToVehicleCoordinates(car_x, car_y, car_yaw, xy_veh[0], xy_veh[1]);
    //xy = mapPtVehCoordToMapCoordinates(car_x, car_y, car_yaw, xy[0], xy[1]);
    next_val_xy.push_back(pveh);
  }
}

void trajectory(std::vector<double> &map_waypoints_s,
    VectorPoints &map_waypoints, VectorPoints &previous_path,
    const int N_samples, double car_s, double car_d, double car_speed,
    VectorPoints &next_val_xy) {
  my_trajectory(map_waypoints_s, map_waypoints, previous_path, N_samples,
      car_s, car_d, car_speed, next_val_xy);

}

