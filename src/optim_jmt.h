/*
 * optim_jmt.h
 *
 *  Created on: 3 Oct 2017
 *      Author: puma
 */

#ifndef OPTIM_JMT_H_
#define OPTIM_JMT_H_

std::vector<double> optim_jmt(double sk, double sk_dot,
		double sk_double_dot, double sT_dot, double sT_double_dot,
		double & sT_optimised, double & T_optimised);



#endif /* OPTIM_JMT_H_ */
