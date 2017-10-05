/*
 * trajectory.h
 *
 *  Created on: Oct 5, 2017
 *      Author: Raphael
 */

#ifndef TRAJECTORY_H_
#define TRAJECTORY_H_



void trajectory(std::vector<double> &previous_path_x, std::vector<double> &previous_path_y, const int N_samples, double car_s, double car_speed,
                std::vector<double> &next_x_vals, std::vector<double> &next_y_vals);


#endif /* TRAJECTORY_H_ */
