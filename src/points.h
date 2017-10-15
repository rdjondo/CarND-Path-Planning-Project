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

// Transform from Frenet s,d coordinates to Cartesian x,y
Point getXY(double s, double d, const std::vector<double> &maps_s,
    const VectorPoints &maps);

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
std::vector<double> getFrenet(double x, double y, double theta,
    const std::vector<double> &maps_x, const std::vector<double> &maps_y);

double polyval(const std::vector<double> &coeffs, double x);

std::vector<double> polyder(const std::vector<double> &coeffs);

#endif /* POINTS_H_ */
