/*
 * trajectory.cpp
 *
 *  Created on: Oct 5, 2017
 *      Author: Raphael
 */



#include <cmath>
#include <iostream>
#include <thread>
#include <vector>

#include "spline.h"
#include "utils.h"
#include "jmt.h"
#include "optim_jmt.h"

using namespace std;


void trajectory(vector<double> &previous_path_x, vector<double> &previous_path_y, const int N_samples, double car_s, double car_speed,
                vector<double> &next_x_vals, vector<double> &next_y_vals){
  // DONE: smoothen road path between the waypoints using Spline

  // TODO: Calculate and control SDC vehicle speed and lane position
  // Use a Jerk minimizing function to control the vehicle speed (use the two past points for continuity)
  // Let's compute the Jerk minimization in the Frenet referential

  const double delta_t = 0.02;
  static vector<double> coeffs_old = {0.0, 0.0 , 0.0 , 0.0 , 0.0 , 0.0};
  static int N_samples_old = -1;

  static double last_s = 1e20; /* initialise to an arbitrary large number*/
  const int previous_path_size = previous_path_x.size();
  double sk = car_s;
  double sk_dot = car_speed/2.23694;
  double sk_double_dot = 0.0;

  double sT_dot = 18;
  //if(((int)floor(car_s/100))%2==0) sT_dot = 20;
  double sT_double_dot = 0.0;

  /* Find delay single last plan */
  double delay_t=0.24;
  if(N_samples_old>=0) {
    double t_incr=0.1;
    int old_dir = 1;
    int dir = 1;
    double s_estim = polyval(coeffs_old, delay_t);
    while(fabs(s_estim-car_s)>0.01 && delay_t>-0.2 ){
      if(s_estim>car_s){
        delay_t -= t_incr;
        dir = 1;
      } else{
        delay_t += t_incr;
        dir = -1;
      }
      s_estim = polyval(coeffs_old, delay_t);
      if(dir != old_dir){
        t_incr/=2.0;
      }
      old_dir = dir;
    }
    if(delay_t<0.0) delay_t = 0.0;

    /* Display estimated telemetry from old plan */
    double last_t = (N_samples-1)*delta_t;
    car_s = last_s;
    double sk = last_s;
    cout<<"last_s:"<<last_s<<"   last_t:"<<last_s<<endl;
    cout<<"len(previous_path_x):"<<previous_path_size<<endl;
    cout<<"car_s:"<<car_s<<"  estim car_s:"<<polyval(coeffs_old, last_t)<<endl;
    vector<double> speed_poly = polyder(coeffs_old);
    sk_dot = polyval( speed_poly, last_t);
    cout<<"car_speed:"<<car_speed<<"  estim car_speed:"<<sk_dot*2.23694<<endl;
    vector<double> acc_poly = polyder(speed_poly);
    sk_double_dot = polyval( acc_poly, last_t);
    cout<<"estim car_acc:"<<sk_double_dot<<endl;
    /* Let's compute the kinematic parameters on the last point */
    cout<<"Previous x path"<<previous_path_x<<" size "<<previous_path_size<<endl;
    //cout<<"Previous y path"<<previous_path_y<<" size "<<previous_path_size<<endl;
  }


  double sT_optimised = 0.0;
  double T_optimised = 0.0;

  vector<double> coeffs = optim_jmt(sk, sk_dot, sk_double_dot, sT_dot,
      sT_double_dot, sT_optimised, T_optimised);

  coeffs_old = coeffs;

  N_samples_old = N_samples;

  cout<<"coeffs = ["<<coeffs[0]<<", "<<coeffs[1]<<", "<<coeffs[2]<<", "<<coeffs[3]
  <<", "<<coeffs[4]<<", "<<coeffs[5]<<"] "<<endl;

  next_x_vals.clear();
  next_y_vals.clear();
  int start_copy = (int)(delay_t/delta_t)+1;
  for(int i = start_copy; i< previous_path_size - 1 && N_samples_old>=0; ++i) {
    double x = previous_path_x[i];
    double y = previous_path_y[i];
    next_x_vals.push_back(x);
    next_y_vals.push_back(y);
  }
  double old_x = next_x_vals.back();
  double old_y = next_y_vals.back();

  // double dist_inc = 0.4;
  double next_s;
  int t_idx = 0;
  for(int i = previous_path_size+1; i < N_samples; ++i, t_idx++;)
  {
    next_s = polyval(coeffs, (t_idx)*delta_t);


    double next_d = 6;
    vector<double> xy_veh = getXY(next_s, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
    //vector<double> xy = mapCoordToVehicleCoordinates(car_x, car_y, car_yaw, xy_veh[0], xy_veh[1]);
    //xy = mapPtVehCoordToMapCoordinates(car_x, car_y, car_yaw, xy[0], xy[1]);
    next_x_vals.push_back(xy_veh[0]);
    next_y_vals.push_back(xy_veh[1]);
  }
  last_s = next_s;

}

