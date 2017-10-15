/*
 * trajectory.h
 *
 *  Created on: Oct 5, 2017
 *      Author: Raphael
 */

#ifndef TRAJECTORY_H_
#define TRAJECTORY_H_

void trajectory(std::vector<double> &map_waypoints_s, VectorPoints &map_waypoints,
    VectorPoints &previous_path, const int N_samples, double car_s, double car_d,
    double car_speed,double car_yaw, VectorPoints & next_vals);


#endif /* TRAJECTORY_H_ */
