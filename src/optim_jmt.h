/*
 * optim_jmt.h
 *
 *  Created on: 3 Oct 2017
 *      Author: rdjondo
 */

#ifndef OPTIM_JMT_H_
#define OPTIM_JMT_H_

/** optim_jmt : This function optimises the jerk for the s coordinate in the Frenet system 
 * Optimize final distance position sT and T so that the jerk
 * is defined by an affine function. This type of affine jerk
 * seems to reduce the envelope of the jerk and the acceleration.
 * The speed tends to not overshoot in this configuration.
 * This is found in my quick experiments but
 * would need to be proven formally :-)
 * The routine uses a concept of virtual acceleration.
 *
 */
std::vector<double> optim_jmt_affine(double sk, double sk_dot,
		double sk_double_dot, double sT_dot, double sT_double_dot,
		double virtual_acceleration);

/** optim_jmt_quadratic : This function optimises the jerk for the d coordinate in the Frenet system
 * is the quadratic Jerk version of optim_jmt_affine
 * The function solves for a minimum quadratic Jerk polynomial.
 */
std::vector<double> optim_jmt_quadratic(double dk, double dk_dot, double dk_double_dot,
    double dT, double dT_dot, double dT_double_dot, double virtual_speed);


#endif /* OPTIM_JMT_H_ */
