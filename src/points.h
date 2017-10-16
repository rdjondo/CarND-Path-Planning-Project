/*
 * points.h
 *
 *  Created on: 7 Oct 2017
 *      Author: puma
 */

#ifndef POINTS_H_
#define POINTS_H_

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

extern double distance(double x1, double y1, double x2, double y2) ;

class Point {
public:
  double x;
  double y;
};

class VectorPoints {
public:
  std::vector<Point> pts;
  void setPoints(const std::vector<double> & x, const std::vector<double> & y);
  void setPoints(const nlohmann::json & x, const nlohmann::json & y);
  VectorPoints();
  VectorPoints& operator = (const VectorPoints &v);
  VectorPoints(const VectorPoints&);
  virtual ~VectorPoints();
public:
  std::vector<double> getVectorX() const;
  std::vector<double> getVectorY() const;
  std::ostream& printCsv(std::ostream &strm, int senquence) const;
  void push_back(Point pt);
  void reserve(size_t size);
  void clear();
  size_t size();
  const Point & at(size_t i);
};

std::ostream& operator<<(std::ostream &strm, const Point &a);

std::ostream& operator<<(std::ostream &strm, const VectorPoints &a);

class RoadGeometry {
private:
  double max_s ; /* Largest S point in map */
  std::vector<double> maps_x ; /* List of x points */
  std::vector<double> maps_y ; /* List of y points */
  std::vector<double> maps_s ; /* List of y points */

  // Defines interpolation functions for smoothing
  // s -> x and s -> y
  tk::spline spline_road_x;
  tk::spline spline_road_y;
  bool splines_initialized = false;

  std::vector<double> spline_s;
  std::vector<double> spline_x;
  std::vector<double> spline_y;

  void init(const RoadGeometry &r);
  int closestWaypoint(double x, double y);
  int nextWaypoint(double x, double y, double theta);

public:
  double getMaxS() const ; /* Return largest S point in map */
  RoadGeometry(const VectorPoints map_waypoints, const std::vector<double> &maps_s);

  RoadGeometry& operator = (const RoadGeometry &r);

  RoadGeometry(const RoadGeometry &r);
  virtual ~RoadGeometry();

// Transform from Frenet s,d coordinates to Cartesian x,y
  Point getXY(double s, double d);

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
  std::vector<double> getFrenet(double x, double y, double theta);

};

double polyval(const std::vector<double> &coeffs, double x);

std::vector<double> polyder(const std::vector<double> &coeffs);

#endif /* POINTS_H_ */
