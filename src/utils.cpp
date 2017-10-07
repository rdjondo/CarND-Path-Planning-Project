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
#include "json.hpp"
#include "spline.h"
#include "utils.h"

using namespace std;

double deg2rad(double x) {
	return x * pi() / 180;
}
double rad2deg(double x) {
	return x * 180 / pi();
}


//////////////////////////////
/** Defining return function to print internals of string class */


void VectorPoints::setPoints(const std::vector<double> & x, const std::vector<double> & y){
	for(size_t i=0; i<x.size() && i<y.size(); ++i){
		Point p = {x[i], y[i]};
		this->pts.push_back(p);
	}
}

void VectorPoints::setPoints(const nlohmann::json & x, const nlohmann::json & y){
	for(size_t i=0; i<x.size() && i<y.size(); ++i){
		Point p = {(double )x[i], (double) y[i]};
		this->pts.push_back(p);
	}
}


std::vector<double> VectorPoints::getVectorX() const {
	std::vector<double> xVec;
	xVec.reserve(this->pts.size());
	for(Point pt : this->pts){
		xVec.push_back(pt.x);
	}
	return xVec;
}

std::vector<double> VectorPoints::getVectorY() const{
	std::vector<double> yVec;
	yVec.reserve(this->pts.size());
	for(Point pt : this->pts){
		yVec.push_back(pt.y);
	}
	return yVec;
}

void VectorPoints::push_back(Point pt) {
	this->pts.push_back(pt);
}

void VectorPoints::clear() {
	this->pts.clear();
}

size_t VectorPoints::size() {
	return this->pts.size();
}

const Point & VectorPoints::at(size_t i){
	return this->pts.at(i);
}

VectorPoints::~VectorPoints(){}

std::ostream& operator<<(std::ostream &strm, const Point &a) {
  return strm << "(" << a.x << "," << a.y << ")";
}

std::ostream& operator<<(std::ostream &strm, const VectorPoints &a) {
	strm << "[";
	for(size_t i=0; i<a.pts.size(); ++i){
		if(i==0) strm << a.pts[i] << ", " ;
		else strm << a.pts[i];
	}
  return strm << "]";
}

//////////////////////////////


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

int closestWaypoint(double x, double y, const vector<double> &maps_x,
		const vector<double> &maps_y) {

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for (size_t i = 0; i < maps_x.size(); i++) {
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

inline int nextWaypoint(double x, double y, double theta,
		const vector<double> &maps_x, const vector<double> &maps_y) {

	int closestWpt = closestWaypoint(x, y, maps_x, maps_y);

	double map_x = maps_x[closestWpt];
	double map_y = maps_y[closestWpt];

	double heading = atan2((map_y - y), (map_x - x));

	double angle = fabs(theta - heading);

	if (angle > pi() / 4) {
		closestWpt++;
	}

	return closestWpt;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta,
		const vector<double> &maps_x, const vector<double> &maps_y) {
	int next_wp = nextWaypoint(x, y, theta, maps_x, maps_y);

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
Point getXY(double s, double d, const vector<double> &maps_s,
		const VectorPoints & mapVec) {

	// Defines interpolation functions for smoothing
	// s -> x and s -> y
	static tk::spline spline_road_x;
	static tk::spline spline_road_y;
	static bool splines_initialized = false;

	static vector<double> spline_s(maps_s);
	static vector<double> maps_x = mapVec.getVectorX();
	static vector<double> maps_y = mapVec.getVectorY();
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

void loadMap(VectorPoints &map_waypoints,
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
		Point pt = {x, y};
		map_waypoints.push_back(pt);
		map_waypoints_s.push_back(s);
		map_waypoints_dx.push_back(d_x);
		map_waypoints_dy.push_back(d_y);
	}
}

void logWaypoints(std::atomic_bool &ready, int max_loops, const VectorPoints & next_vals) {
	VectorPoints log;

	for (int i = 0; i < max_loops; ++i) {
		if (ready) {
			ready = false;
			for (Point p : log.pts) {
				log.pts.push_back(p);
      }
    } else{
      this_thread::sleep_for(chrono::milliseconds(15));
    }
  }

  ofstream logfile;
  logfile.open ("example.csv");
  logfile<<"X, Y";
  for (size_t i = 0; i < log.pts.size(); ++i) {
	  logfile << log << "\n";
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
		for (size_t deg = 1; deg < coeffs.size(); ++deg) {
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
		for(size_t deg=1; deg<coeffs.size(); ++deg) {
			Df_coeffs.push_back(deg * coeffs[deg]) ;
		}
	} else{
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

