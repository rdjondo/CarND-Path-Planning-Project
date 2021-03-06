/*
 * point.cpp
 *
 *  Created on: 7 Oct 2017
 *      Author: rdjondo
 */

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
#include <string>
#include <mutex>
#include <atomic>
#include <iomanip>
#include "json.hpp"
#include "spline.h"
#include "points.h"

using namespace std;

double deg2rad(double x) {
  return x * pi() / 180;
}
double rad2deg(double x) {
  return x * 180 / pi();
}

//////////////////////////////
/** Defining return function to print internals of string class */

std::vector<double> VectorPoints::getVectorX() const {
  std::vector<double> xVec;
  xVec.reserve(this->pts.size());
  for (Point pt : this->pts) {
    xVec.push_back(pt.x);
  }
  return xVec;
}

std::vector<double> VectorPoints::getVectorY() const {
  std::vector<double> yVec;
  yVec.reserve(this->pts.size());
  for (Point pt : this->pts) {
    yVec.push_back(pt.y);
  }
  return yVec;
}

void VectorPoints::push_back(Point pt) {
  this->pts.push_back(pt);
}

void VectorPoints::reserve(size_t size){
  this->pts.reserve(size);
}
void VectorPoints::clear() {
  this->pts.clear();
}

size_t VectorPoints::size() {
  return this->pts.size();
}

const Point & VectorPoints::at(size_t i) {
  return this->pts.at(i);
}

void VectorPoints::setPoints(const std::vector<double> & x,
    const std::vector<double> & y) {
  for (size_t i = 0; i < x.size() && i < y.size(); ++i) {
    Point p = { x[i], y[i] };
    this->pts.push_back(p);
  }
}

void VectorPoints::setPoints(const nlohmann::json & x,
    const nlohmann::json & y) {
  for (size_t i = 0; i < x.size() && i < y.size(); ++i) {
    Point p = { (double) x[i], (double) y[i] };
    this->pts.push_back(p);
  }
}

VectorPoints::VectorPoints() {
  this->pts.clear();
}


VectorPoints & VectorPoints::operator= (const VectorPoints &v)
{
   this->pts = v.pts;
   return *this;
}

VectorPoints::VectorPoints(const VectorPoints& v) : pts(v.pts){
}

VectorPoints::~VectorPoints() {
}


std::ostream& operator<<(std::ostream &strm, const Point &p) {
  return strm << "(" << p.x << "," << p.y << ")";
}

std::ostream& VectorPoints::printCsv(std::ostream &strm, int senquence) const {
  for (size_t i = 0; i < pts.size(); ++i) {
    strm << senquence << "," << setprecision(9) << pts[i].x << "," << pts[i].y << "\n";
  }
  return strm;
}

std::ostream& operator<<(std::ostream &strm, const VectorPoints &a) {
  strm << "[";
  for (size_t i = 0; i < a.pts.size(); ++i) {
    if (i == 0)
      strm << a.pts[i] << ", ";
    else
      strm << a.pts[i];
  }
  return strm << "]";
}

double distance(double x1, double y1, double x2, double y2) {
  return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

int RoadGeometry::closestWaypoint(double x, double y) {

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

int RoadGeometry::nextWaypoint(double x, double y, double theta) {

  int closestWpt = this->closestWaypoint(x, y);

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
vector<double> RoadGeometry::getFrenet(double x, double y, double theta) {
  int next_wp = this->nextWaypoint(x, y, theta);

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

RoadGeometry::RoadGeometry(const VectorPoints map_waypoints, const std::vector<double> &maps_s) {

  // Defines interpolation functions for smoothing
  // s -> x and s -> y

  if (!splines_initialized) {
    // Add additional point at the end of the spline
    // that sort of maps to the second point to smooth
    // out the derivatives

    this->maps_s = maps_s;
    this->spline_s= maps_s;
    this->maps_x = map_waypoints.getVectorX();
    this->maps_y = map_waypoints.getVectorY();
    spline_x = maps_x;
    spline_y= maps_y;
    double last_s;
    for(size_t i=0; i<3; i++){
      if(i==0){
        double dx_wrap = maps_x[0]-maps_x.back();
        double dy_wrap = maps_y[0]-maps_y.back();
        last_s = maps_s.back() + sqrt(dx_wrap*dx_wrap+dy_wrap+dy_wrap);
        max_s = last_s;
      } else{
        last_s = spline_s.back() + maps_s[i];
      }
      spline_s.push_back(last_s );
      spline_x.push_back(maps_x[i]);
      spline_y.push_back(maps_y[i]);
    }
    spline_road_x.set_points(spline_s, spline_x);
    spline_road_y.set_points(spline_s, spline_y);
    splines_initialized = true;
  }
}

void RoadGeometry::init(const RoadGeometry& r){
  this->maps_s = r.maps_s;
  this->maps_x = r.maps_x;
  this->maps_y = r.maps_y;
  this->max_s = r.max_s;
  this->spline_road_x = r.spline_road_x;
  this->spline_road_y = r.spline_road_y;
  this->spline_s = r.spline_s;
  this->spline_x = r.spline_x;
  this->spline_y = r.spline_y;
  this->splines_initialized = r.splines_initialized;
}
// Constructor
RoadGeometry::RoadGeometry(const RoadGeometry& r){
  this->init(r);
}

// Destructor
RoadGeometry::~RoadGeometry(){}


/* Return largest S point in map */
double RoadGeometry::getMaxS() const {
  return max_s;
}

// Assignment operator
RoadGeometry& RoadGeometry::operator = (const RoadGeometry &r){
  this->init(r);
  return *this;
}


// Transform from Frenet s,d coordinates to Cartesian x,y
Point RoadGeometry::getXY(double s, double d) {

  // Makes sure that the s requested never exceeds the
  // original maximal value of s.
  // Using modulo to perform the loop around s values.
  if(s> max_s){
    s = fmod(s, max_s);
  }
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
  /* Computing 1st derivative of the polynomial */
  vector<double> Df_coeffs;
  if (coeffs.size() > 0) {
    Df_coeffs.reserve(coeffs.size() - 1);
    for (size_t deg = 1; deg < coeffs.size(); ++deg) {
      Df_coeffs.push_back(deg * coeffs[deg]);
    }
  } else {
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

