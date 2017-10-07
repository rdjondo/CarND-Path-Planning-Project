/*
 * utils.h
 *
 *  Created on: 30 Sep 2017
 *      Author: puma
 */

#ifndef UTILS_H_
#define UTILS_H_


// In the Cygwin environment M_PI is not defined.
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

constexpr double pi() {
	return M_PI;
}

// For converting back and forth between radians and degrees.
extern double deg2rad(double x);
extern double rad2deg(double x);

extern std::string hasData(std::string s);

class Point{
public:
	double x;
	double y;
};


class VectorPoints{
public:
	std::vector<Point> pts;
	void setPoints(const std::vector<double> & x, const std::vector<double> & y);
	void setPoints(const nlohmann::json & x, const nlohmann::json & y);
	virtual ~VectorPoints();
public:
	std::vector<double> getVectorX() const;
	std::vector<double> getVectorY() const;
	void push_back(Point pt);
	void clear();
	size_t size();
	const Point & at(size_t i);
};

void logWaypoints(std::atomic_bool & ready, int max_loops,
		const VectorPoints & next_vals);

// Transform from Frenet s,d coordinates to Cartesian x,y
Point getXY(double s, double d, const std::vector<double> &maps_s,
		const VectorPoints &maps);

void loadMap(VectorPoints &map_waypoints,
		std::vector<double> &map_waypoints_s,
		std::vector<double> &map_waypoints_dx,
		std::vector<double> &map_waypoints_dy);

double polyval(const std::vector<double> &coeffs, double x) ;

std::vector<double> polyder(const std::vector<double> &coeffs);

#endif /* UTILS_H_ */
