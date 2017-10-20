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
#include "jmt.h"
#include "trajectory.h"
#include "optim_jmt.h"

using namespace std;

/** This function implement an ideal trajectory that was useful for debugging 
*  trajectory closing-loop issues.
*/
static void ideal_trajectory(RoadGeometry &road, VectorPoints &previous_path,
    const int N_samples, double car_s, double car_d, double car_speed,
    VectorPoints &next_vals) {
  double dist_inc = 2;
  static std::vector<double> next_s_vec;

  /* Define next initial position index values */
  double init_s = -1e9;
  size_t init_index = 0;

  static bool isInitialised = false;

  if(isInitialised){
    /* Linear search for initial s point to send back to simulator (motion smoothing)*/
    if(next_s_vec[0]>road.getMaxS()){
      for(size_t i = 0; i<next_s_vec.size(); ++i){
        next_s_vec[i] = fmod(next_s_vec[i],road.getMaxS());
      }
    }
    /* Bug here because when closing loop init_s is bigger than car_s */
    for(init_index=0 ;init_s < car_s && init_index<previous_path.size(); ++init_index){
      init_s = next_s_vec[init_index];
      init_s = fmod(init_s, road.getMaxS());
    }
  } else {
    init_s = car_s;
    isInitialised = true;
  }

  next_s_vec.clear();

  for (int i = 0; i < N_samples; i++) {
    double next_s = init_s + (i + 1) * dist_inc;
    next_s_vec.push_back(next_s);
    double next_d = 6;
    Point pt = road.getXY(next_s, next_d);
    next_vals.push_back(pt);
  }
}

  /** my_trajectory: smoothen road path between the waypoints using JMT
   * Jerk Minimized trajectory
  */
static void my_trajectory(RoadGeometry &road, VectorPoints &previous_path,
    const int N_samples, double car_s, double car_d, double car_speed,
    double target_car_speed, double target_car_d, VectorPoints &next_val_xy) {

  // This calculates and control SDC vehicle speed and lane position
  // Use a Jerk minimizing function to control the vehicle speed.
  // It uses the previously calculated polynomial for continuity.
  // The Jerk minimization is computed in the Frenet coordinate system

  const double delta_t = 0.02;
  static vector<double> coeff_s_old = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
  static vector<double> coeff_d_old = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
  static std::vector<double> next_s_vec;
  static std::vector<double> next_d_vec;

  double sk;
  double sk_dot;
  double sk_double_dot = 0.0;

  double sT_dot = target_car_speed;
  double sT_double_dot = 0.0;

  double dk;
  double dk_dot;
  double dk_double_dot = 0.0;

  double dT_dot = 0.0;
  double dT_double_dot = 0.0;


  /* Define next initial position index values */
  static double init_s = -1e9;
  double init_d = -1e9;
  int init_index = 0;


  if (next_s_vec.size() > 1) {

    /*  Wrap points in case of loop closure */
    if(next_s_vec.back() > road.getMaxS() || next_s_vec[0]>road.getMaxS() ){
      for(size_t i = 0; i<next_s_vec.size(); ++i){
        next_s_vec[i] = fmod(next_s_vec[i],road.getMaxS());
      }
    }

    /* Nearest neighbour to find index corresponding to S frenet coordinate
     * This is useful to find the current time index */
    double smallest_distance = 1e10;
    for (int i = 1; i < next_s_vec.size(); ++i) {
      double dist = fabs(car_s - (next_s_vec[i-1] + next_s_vec[i])/2 );
      if(dist<smallest_distance){
        smallest_distance = dist;
        init_index = i-1;
      }
    }

    init_s = next_s_vec[init_index];
    init_d = next_d_vec[init_index];

  } else {
    init_index = 0;
    init_s = car_s;
    init_d = car_d;
    sk_dot = car_speed / 2.23694; /* Comvert Mi/hr to m/s */

    /* Initial position */
    coeff_s_old[0] = init_s;
    coeff_d_old[0] = init_d;

    /* Initial speed */
    coeff_s_old[1] = sk_dot;
  }

  /* New time for estimating the computation lag */
  double delay_t = ((double)init_index ) * delta_t;

  /* Display estimated telemetry from old plan Frenet S axis  */
  sk = init_s;
  vector<double> speed_poly_s = polyder(coeff_s_old);
  sk_dot = polyval(speed_poly_s, delay_t);
  vector<double> acc_poly_s = polyder(speed_poly_s);
  sk_double_dot = polyval(acc_poly_s, delay_t);

  double dT = target_car_d;

  /* Display estimated telemetry from old plan for Frenet D axis */
  dk = init_d;
  cout<<"delay_t :"<<delay_t<< "  ";
  vector<double> speed_poly_d = polyder(coeff_d_old);
  dk_dot = polyval(speed_poly_d, delay_t);
  vector<double> acc_poly_d = polyder(speed_poly_d);
  dk_double_dot = polyval(acc_poly_d, delay_t);


  double virtual_acceleration = 2.0; /* virtual acceleration in m/s^2 */
  vector<double> coeff_s = optim_jmt_affine(sk, sk_dot, sk_double_dot, sT_dot,
      sT_double_dot, virtual_acceleration);

  double virtual_speed = 0.6; /* virtual speed in m/s */
  vector<double> coeff_d = optim_jmt_quadratic( dk,  dk_dot,  dk_double_dot,
      dT,  dT_dot,  dT_double_dot, virtual_speed);

  /* Remember calculated motion polynomial for continuity in next round */
  coeff_s_old = coeff_s;
  coeff_d_old = coeff_d;

  cout << "coeffs = [" << coeff_s[0] << ", " << coeff_s[1] << ", " << coeff_s[2]
      << ", " << coeff_s[3] << ", " << coeff_s[4] << ", " << coeff_s[5] << "] "
      << endl;

  next_val_xy.clear();
  next_s_vec.clear();
  next_d_vec.clear();

  for (int ti = 0; ti < N_samples; ++ti) {
    double next_s = polyval(coeff_s, (ti+1.0) * delta_t);
    double next_d = polyval(coeff_d, (ti+1.0) * delta_t);
    next_s_vec.push_back(next_s);
    next_d_vec.push_back(next_d);

    Point pveh = road.getXY(next_s, next_d);
    next_val_xy.push_back(pveh);
  }
}

void trajectory(RoadGeometry &road, VectorPoints &previous_path,
    const int N_samples, double car_s, double car_d, double car_speed,
    double target_car_speed, double target_car_d, VectorPoints &next_val_xy) {
  my_trajectory(road, previous_path, N_samples, car_s, car_d, car_speed,
      target_car_speed, target_car_d, next_val_xy);
//  ideal_trajectory(road, previous_path, N_samples,
//      car_s, car_d, car_speed, next_val_xy);

}

