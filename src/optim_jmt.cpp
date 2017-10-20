/*
 * optim_jmt.cpp
 *
 *  Created on: 3 Oct 2017
 *      Author: rdjondo
 */
#include <vector>
#include <cmath>
#include <iostream>
#include <algorithm>    // For std::max() function
#include <mutex>
#include "json.hpp"
#include "jmt.h"
#include "optim_jmt.h"

using namespace std;

/**
 *  Optimize final distance position sT and T so that the jerk
 * is defined by an affine function. This type of affine jerk
 * seems to reduce the envelope of the jerk and the acceleration.
 * The speed tends to not overshoot in this configuration.
 * This is found in my quick experiments but
 * would need to be proven formally :-)
 * The routine uses a concept of virtual acceleration.
 * The equivalent virtual acceleration corresponds to
 * the equivalent acceleration that would satisfy the
 * speed constraints
 * 
 * sk is the initial lateral position
 * sk_dot is the initial speed
 * sk_dot is the initial acceleration
 * sT is the final lateral position
 * sT_dot is the final speed
 * sT_double_dot is the final acceleration
 * virtual_acceleration is the equivalent acceleration of the planned motion
 */
std::vector<double> optim_jmt_affine(double sk, double sk_dot,
    double sk_double_dot, double sT_dot, double sT_double_dot,
    double virtual_acceleration) {

  /* Avoids divisions by zero */
  if(fabs(virtual_acceleration)<1e-2){
    virtual_acceleration = 1.0; /* m/s^2 */
  }

  const double T = min(20.0,
      max(2.0, fabs(sT_dot - sk_dot) / virtual_acceleration));

  vector<double> start;
  vector<double> end ;
  vector<double> coeffs ;
  coeffs.reserve(6);

  start = {sk, sk_dot, sk_double_dot};
  end = {0.0, sT_dot, sT_double_dot};
  coeffs = JMT_affine(start, end, T);

  return coeffs;
}

/** optim_jmt_quadratic : is the quadratic Jerk version of optim_jmt_affine
 * The function solves for a minimum quadratic Jerk polynomial.
 *
 * dk is the initial lateral position
 * dk_dot is the initial speed
 * dk_dot is the initial acceleration
 * dT is the final lateral position
 * dT_dot is the final speed
 * dT_double_dot is the final acceleration
 * T is the final time of the planned motion
 */
std::vector<double> optim_jmt_quadratic(double dk, double dk_dot, double dk_double_dot,
    double dT, double dT_dot, double dT_double_dot, double virtual_speed){

  vector<double> start = { dk, dk_dot, dk_double_dot };
  vector<double> end = { dT, dT_dot, dT_double_dot };

  /* Avoids divisions by zero */
  if(fabs(virtual_speed)<1e-2){
    virtual_speed = 1e-2; /* m/s */
  }
  
  const double T = max(4.0, min(20.0, fabs(dk - dT)/ virtual_speed));

  vector<double> coeff_d = JMT(start, end, T);
  return coeff_d;
}

