/*
 * optim_jmt.cpp
 *
 *  Created on: 3 Oct 2017
 *      Author: puma
 */
#include <vector>
#include <cmath>
#include <iostream>
#include <algorithm>    // For std::max() function
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
 * The routine uses a concept of virtual acceleration that
 * must have been used somewhere else
 */
std::vector<double> optim_jmt(double sk, double sk_dot,
		double sk_double_dot, double sT_dot, double sT_double_dot,
		double & sT_optimised, double & T_optimised) {

	// Make static
	double sT = sk + 128;

	/* OPTIMISATION ROUTINE (DICHOTOMY)
	 * The position function returned by the JMT function is a polynomial
	 * of degree 5.
	 * So, the jerk is a quadratic function. This routine get the quadratic
	 * coefficient of coeff as close to zero as possible.
	 * Increase sT when on jerk(t), a5 is negative (and -a4/(2*a5) is between 0
	 * and T)
	 * Reduce sT when jerk on jerk(t), a5 is positive and (-a4/(2*a5) is between
	 * 0 and T)
	 * Stop when a5 is very small
	 */

	/* The equivalent virtual acceleration corresponds to
	 * the equivalent acceleration that would satisfy the
	 * speed constraints */
	const double virtual_acceleration = 1.0; /* m/s^2 */

	const double T = min(40.0, max(2.0, fabs(sT_dot-sk_dot)/virtual_acceleration));

	int num_iter = 1;

	/* Call JMT function to find polynomial that satisfies the
	 * boundary conditions */
	vector<double> start = { sk, sk_dot, sk_double_dot };
	vector<double> end = { sT, sT_dot, sT_double_dot };
  vector<double> coeffs = JMT(start, end, T);

  // This is the search increment
  double sT_inc = (sT-sk)/2;
  int direction = 1;
  int direction_change = 1;

  while(fabs(coeffs[5])>10e-6){
  	++num_iter;
  	//cout<<"Num iter:"<<num_iter<<"  coeffs_6:"<<coeffs[5]<<endl;

    // jerk(t) is a quadratic function
    if(coeffs[5]<0){
      /* Increase sT when on jerk(t), a5 is negative (and -a4/(2*a5) is between 0
       and T) */
      direction_change = direction_change * direction;
      direction = 1;
    }
    else{
      /* Reduce sT when jerk on jerk(t), a5 is positive and (-a4/(2*a5) is between
       0 and T */
      direction_change = direction_change * direction;
      direction = -1;
    }

    if (direction_change<0)
      sT_inc = sT_inc/2;

    /* Change terminal position */
    sT = sT + sT_inc*direction;

    start = { sk, sk_dot, sk_double_dot };
    end = { sT, sT_dot, sT_double_dot };

    coeffs = JMT(start, end, T);

    if (num_iter>=50){
       break;
    }

  } /* end while */
	//cout<<"Num iter:"<<num_iter<<"  coeffs_6:"<<coeffs[5]<<endl;

  T_optimised = T;
  sT_optimised = sT;
  return coeffs;
}

