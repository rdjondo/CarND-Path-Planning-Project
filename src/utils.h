/*
 * utils.h
 *
 *  Created on: 30 Sep 2017
 *      Author: puma
 */

#ifndef UTILS_H_
#define UTILS_H_

// For converting back and forth between radians and degrees.
constexpr double pi() {
	return M_PI;
}

extern double deg2rad(double x);
extern double rad2deg(double x);

extern std::string hasData(std::string s);

double distance(double x1, double y1, double x2, double y2);

int ClosestWaypoint(double x, double y, const std::vector<double> &maps_x,
		const std::vector<double> &maps_y);

std::vector<double> getFrenet(double x, double y, double theta,
		const std::vector<double> &maps_x, const std::vector<double> &maps_y);

// Transform from Frenet s,d coordinates to Cartesian x,y
std::vector<double> getXY(double s, double d, const std::vector<double> &maps_s,
		const std::vector<double> &maps_x, const std::vector<double> &maps_y);

void load_map(std::vector<double> &map_waypoints_x,
		std::vector<double> &map_waypoints_y,
		std::vector<double> &map_waypoints_s,
		std::vector<double> &map_waypoints_dx,
		std::vector<double> &map_waypoints_dy);

double polyval(const std::vector<double> &coeffs, double x) ;

std::vector<double> polyder(const std::vector<double> &coeffs);

#endif /* UTILS_H_ */
