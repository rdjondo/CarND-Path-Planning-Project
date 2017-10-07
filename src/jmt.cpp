/*
 * jmt.cpp
 *
 *  Created on: 1 Oct 2017
 *      Author: rdjondo
 */

#include <cmath>
#include <vector>
#include "Eigen-3.3/Eigen/Dense"
#include "jmt.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

vector<double> JMT(vector<double> &start, vector<double> &end, double T) {
  /*
   Calculate the Jerk Minimizing Trajectory that connects the initial state
   to the final state in time T.

   INPUTS

   start - the vehicles start location given as an array of length 4
   corresponding to initial values of [s, s_dot, s_double_dot]

   end   - the desired end state for vehicle. Like "start" this is a
   length three array.

   T     - The duration, in seconds, over which this manoeuvre should occur.

   If isJerkDefined is false, then solves system assuming jerk is unknown.
   Otherwise, we assume that s(T) is unknown but jerk is known and defined.
   In this last case


   OUTPUT
   an array of length 6, each value corresponding to a coefficient in the polynomial
   s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5

   EXAMPLE

   > JMT( [0, 10, 0], [10, 10, 0], 1)
   [0.0, 10.0, 0.0, 0.0, 0.0, 0.0]
   */

  /*
   The Jerk Minimising function must be a 5 degree polynomial

   The equations for position, velocity, and acceleration are given by if jerk is unknown:

   s(t)   = sk + sk_dot * t + sk_double_dot/2 * t**2 + a3 * t**3     + a4 * t**4      + a5 * t**​5
   s'(t)  = 0  + sk_dot     + sk_double_dot * t      + a3 * 3 * t**2 + a4 * 4 * t**3  + a5 * 5 * t**​4
   s''(t) = 0  + 0          + sk_double_dot          + a3 * 6 * t    + a4 * 12 * t**2 + a5 * 20 * t**​3

   so when re-arranging the terms, for t == T, the end of the motion
   s(T)   - ( sk + sk_dot * T + sk_double_dot/2 * T**2 ) == a3 * T**3     + a4 * T**4      +  a5 * T**​5
   s'(T)  - ( 0  + sk_dot     + sk_double_dot   * T    ) == a3 * 3 * T**2 + a4 * 4 * T**3  +  a5 * 5 * T**​4
   s''(T) - ( 0  + 0          + sk_double_dot          ) == a3 * 6 * T    + a4 * 12 * T**2 +  a5 * 20 * T**​3

   s_lim = Tpk * ak

   */

  double sk = start[0];
  double sk_dot = start[1];
  double sk_double_dot = start[2];

  double sT = end[0];
  double sT_dot = end[1];
  double sT_double_dot = end[2];

  VectorXd s_lim(3);
  MatrixXd Tpk(3, 3);
  VectorXd ak(3);

  s_lim << sT - (sk + sk_dot * T + sk_double_dot / 2 * T * T), sT_dot
      - (0 + sk_dot + sk_double_dot * T), sT_double_dot
      - (0 + 0 + sk_double_dot);

  Tpk << T * T * T, T * T * T * T, T * T * T * T * T, 3 * T * T, 4 * T * T * T, 5
      * T * T * T * T, 6 * T, 12 * T * T, 20 * T * T * T;

  ak = Tpk.colPivHouseholderQr().solve(s_lim);
  vector<double> coeff = { sk, sk_dot, sk_double_dot / 2, ak(0), ak(1), ak(2) };

  return coeff;

}
