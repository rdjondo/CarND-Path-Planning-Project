/*
 * utils.cpp
 *
 *  Created on: 30 Sep 2017
 *      Author: puma
 */

// Load up map values for waypoint's x,y,s and d normalized normal vectors
#include <cmath>
#include <fstream>
#include <sstream>
#include <iostream>
#include <vector>
#include <atomic>
#include <thread>
#include <chrono>
#include <string>
#include "spline.h"
#include "utils.h"

using namespace std;

double deg2rad(double x) {
	return x * pi() / 180;
}
double rad2deg(double x) {
	return x * 180 / pi();
}

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

double distance(double x1, double y1, double x2, double y2) {
	return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

int ClosestWaypoint(double x, double y, const vector<double> &maps_x,
		const vector<double> &maps_y) {

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for (int i = 0; i < maps_x.size(); i++) {
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x, y, map_x, map_y);
		if (dist < closestLen) {
			closestLen = dist;
			closestWaypoint = i;
		}
	}

	return closestWaypoint;
}

inline int NextWaypoint(double x, double y, double theta,
		const vector<double> &maps_x, const vector<double> &maps_y) {

	int closestWaypoint = ClosestWaypoint(x, y, maps_x, maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2((map_y - y), (map_x - x));

	double angle = fabs(theta - heading);

	if (angle > pi() / 4) {
		closestWaypoint++;
	}

	return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta,
		const vector<double> &maps_x, const vector<double> &maps_y) {
	int next_wp = NextWaypoint(x, y, theta, maps_x, maps_y);

	int prev_wp;
	prev_wp = next_wp - 1;
	if (next_wp == 0) {
		prev_wp = maps_x.size() - 1;
	}

	double n_x = maps_x[next_wp] - maps_x[prev_wp];
	double n_y = maps_y[next_wp] - maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x * n_x + x_y * n_y) / (n_x * n_x + n_y * n_y);
	double proj_x = proj_norm * n_x;
	double proj_y = proj_norm * n_y;

	double frenet_d = distance(x_x, x_y, proj_x, proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000 - maps_x[prev_wp];
	double center_y = 2000 - maps_y[prev_wp];
	double centerToPos = distance(center_x, center_y, x_x, x_y);
	double centerToRef = distance(center_x, center_y, proj_x, proj_y);

	if (centerToPos <= centerToRef) {
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for (int i = 0; i < prev_wp; i++) {
		frenet_s += distance(maps_x[i], maps_y[i], maps_x[i + 1], maps_y[i + 1]);
	}

	frenet_s += distance(0, 0, proj_x, proj_y);

	return {frenet_s, frenet_d};
}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s,
		const vector<double> &maps_x, const vector<double> &maps_y) {

	// Defines interpolation functions for smoothing
	// s -> x and s -> y
	// The objects are defined as static so that
	// the parameters are only added once
	static tk::spline spline_road_x;
	static tk::spline spline_road_y;
	static bool splines_initialized = false;

	static vector<double> spline_s(maps_s);
	static vector<double> spline_x(maps_x);
	static vector<double> spline_y(maps_y);

	if (!splines_initialized) {
		// Add additional point at the end of the spline
		// that sort of maps to the second point to smooth
		// out the derivatives
		spline_s.push_back(maps_x[1] + maps_s.back());
		spline_x.push_back(maps_x[1]);
		spline_y.push_back(maps_y[1]);

		spline_road_x.set_points(maps_s, maps_x);
		spline_road_y.set_points(maps_s, maps_y);
		splines_initialized = true;
	}

	double max_s = maps_s.back();

	// Makes sure that the s requested never exceeds the
	// original maximal value of s.
	// Using modulo to perform the loop around s values.
	s = fmod(s, max_s);

	double eps = 1e-3;

	double x = spline_road_x(s);
	double y = spline_road_y(s);

	double dx_ds = (spline_road_x(s + eps) - x) / eps;
	double dy_ds = (spline_road_y(s + eps) - y) / eps;

	/*
	 * Now let's find how to translate laterally the car by d
	 */

	// Given the map we know that the norm of the heading of the spline cannot be 0
	// because there is no stop point
	double norm_inv = 1 / sqrt(dx_ds * dx_ds + dy_ds * dy_ds);

	// This vector represents the heading of the vehicle with a norm that equals d
	vector<double> heading = { dx_ds * norm_inv * d, dy_ds * norm_inv * d };

	// Let's rotate the heading by 90 degrees to find the
	vector<double> normal = { heading[1], -heading[0] };

	return {x + normal[0], y+normal[1]};
}

void load_map(vector<double> &map_waypoints_x, vector<double> &map_waypoints_y,
		vector<double> &map_waypoints_s, vector<double> &map_waypoints_dx,
		vector<double> &map_waypoints_dy) {

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
		map_waypoints_x.push_back(x);
		map_waypoints_y.push_back(y);
		map_waypoints_s.push_back(s);
		map_waypoints_dx.push_back(d_x);
		map_waypoints_dy.push_back(d_y);
	}
}

void log_waypoints(std::atomic_bool &ready, int max_loops,
		const std::vector<double> & next_x_vals,
		const std::vector<double> & next_y_vals) {
	std::vector<double> log_x_vals(1000);
	std::vector<double> log_y_vals(1000);

  for (int i = 0; i < max_loops; ++i) {
    if(ready){
      ready = false;
      for (double x : next_x_vals) {
        log_x_vals.push_back(x);
      }
      for (double y : next_y_vals) {
        log_y_vals.push_back(y);
      }
    } else{
      this_thread::sleep_for(chrono::milliseconds(15));
    }
  }

  ofstream logfile;
  logfile.open ("example.csv");
  logfile<<"X, Y";
  for (int i = 0; i < log_x_vals.size(); ++i) {
	  logfile << log_x_vals[i] <<  ", " <<  log_x_vals[i] << "\n";
  }
  logfile.close();

}

/**
 *  Computing polynomial evaluation using coefficients [a0, a1, .. an] on x :
 *  f(x) = a0 + a1 *x + .. + an * x^n
 */
double polyval(const vector<double> &coeffs, double x) {
	double f = 0.0;

	if (coeffs.size() > 0) {
		f = coeffs[0];
		double x_power = x;
		for (int deg = 1; deg < coeffs.size(); ++deg) {
			f += coeffs[deg] * x_power;
			x_power = x_power * x;
		}
	}
	return f;
}

vector<double> polyder(const vector<double> &coeffs) {
	/* Computing 1st derivative evaluation on the polynomial */
	vector<double>  Df_coeffs;
	if(coeffs.size()>0){
		Df_coeffs.reserve(coeffs.size()-1);
		for(int deg=1; deg<coeffs.size(); ++deg) {
			Df_coeffs.push_back(deg * coeffs[deg]) ;
		}
	} else{
		Df_coeffs.reserve(1);
		Df_coeffs.push_back(0.0);
	}
	return Df_coeffs;
}


// Transform waypoints in map coordinates to vehicle coordinates.
inline vector<double> mapCoordToVehicleCoordinates(double car_x, double car_y,
    double yaw, double x_way, double y_way) {

  // Fix new coordinate axis origin to vehicle's coordinates
  double x = x_way - car_x;
  double y = y_way - car_y;

  // Limit yaw angle

  // Rotate coordinates to vehicle's coordinates
  double x_veh = x * cos(yaw) - y * sin(yaw);
  double y_veh = x * sin(yaw) + y * cos(yaw);
  return
  { x_veh, y_veh};
}

// Transform waypoints in vehicle coordinates to map coordinates.
inline vector<double> mapPtVehCoordToMapCoordinates(double car_x, double car_y,
    double yaw, double x_way, double y_way) {

  // Rotate coordinates to vehicle's coordinates
  double x = x_way * cos(-yaw) - y_way * sin(-yaw);
  double y = x_way * sin(-yaw) + y_way * cos(-yaw);

  // Fix new coordinate axis origin to map's coordinates
  double x_map = x + car_x;
  double y_map = y + car_y;

  return
  { x_map, y_map};
}

