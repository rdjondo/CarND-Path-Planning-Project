/*
 * jmt.h
 *
 *  Created on: 1 Oct 2017
 *      Author: rdjondo
 */

#ifndef JMT_H_
#define JMT_H_
std::vector<double> JMT_affine(std::vector<double> &start,
    std::vector<double> &end, double T);

std::vector<double> JMT(std::vector<double> &start, std::vector<double> &end,
    double T);

#endif /* JMT_H_ */
