/*
 * planner.cpp
 *
 *  Created on: 16 Oct 2017
 *      Author: rdjondo
 */

#include <cmath>
#include <mutex>
#include <vector>
#include <algorithm>    // For std::min() function
#include "json.hpp"
#include "spline.h"
#include "points.h"
#include "utils.h"
#include "planner.h"

using namespace std;

DrivingState::DrivingState(RoadGeometry & road_init) {
  sensor_fusion.clear();
  road = &road_init;
  current_state = KEEP_LANE;
  next_state = KEEP_LANE;
  car_x = 0.0;
  car_y = 0.0;
  car_s = 0.0;
  car_d = 0.0;
  car_yaw = 0.0;
  car_speed = 0.0;
  target_car_speed = 0.1;
  target_car_d = 6.0;
}

DrivingState::~DrivingState() {
}


double DrivingState::getKeepLaneCost() {
  int vehicle_lead = getVehicleLeadIdx(car_d);
  double dist = 1e6;
  double time_to_collision = 1000;

  if (vehicle_lead >= 0) {
    SensorFusion veh = sensor_fusion[vehicle_lead];
    dist = fabs(veh.s - car_s);
    double delta_speed = std::min(0.0,
        car_speed - sqrt(veh.vx * veh.vx + veh.vy * veh.vy));
    time_to_collision = (dist) / (1e-2 + delta_speed);
  }
  double cost = dist + time_to_collision;
  return cost;
}

double DrivingState::getPrepareChangingLeftCost() {
  double dist = 1e6;
  double time_to_collision = 1000;

  int vehicle_lead = getVehicleLeadIdx(car_d  - 4.0 );
  int vehicle_follow = getVehicleFollowingIdx(car_d - 4.0);

  if (vehicle_lead >= 0) {
    SensorFusion veh = sensor_fusion[vehicle_lead];
    dist = fabs(veh.s - car_s);
    double delta_speed = std::min(0.0,
        car_speed - sqrt(veh.vx * veh.vx + veh.vy * veh.vy));
    time_to_collision = (dist) / (1e-2 + delta_speed);
  }
  double cost = dist + time_to_collision;

  if (vehicle_follow >= 0) {
    SensorFusion veh = sensor_fusion[vehicle_lead];
    dist = fabs(veh.s - car_s);
    double delta_speed = std::min(0.0,
        car_speed - sqrt(veh.vx * veh.vx + veh.vy * veh.vy));
    time_to_collision = (dist) / (1e-2 + delta_speed);
  }
  cost = cost + dist + time_to_collision;

  return cost;
}

void DrivingState::nextState() {

  double prepKeepLaneCost = getKeepLaneCost();
  double prepChangeLeftCost = getPrepareChangingLeftCost();
  //double changeLeftCost = getChangingLeftCost();
  //double prepChangeRightCost = getPrepareChangingRightCost();

  double cost;

  switch (current_state) {
    case KEEP_LANE: {

      /* TARGETS
       * 2 possible targets:  speed = max speed or speed = matching speed of vehicle in front
       *
       * By default, use Max speed */
      target_car_speed = 21.0;

      int vehicle_lead = getVehicleLeadIdx(car_d);
      /* Assign its speed as target speed */
      SensorFusion veh = sensor_fusion[vehicle_lead];
      if (vehicle_lead >= 0) {
        target_car_speed = sqrt(veh.vx * veh.vx + veh.vy * veh.vy)*0.98;
        if(veh.s - car_s<10) target_car_speed = max(0.0, target_car_speed - 2.0);
      }
      /**
       * TRANSITIONS :
       *   KEEP_LANE
       *   PREPARE_CHANGING_LEFT,
       *   PREPARE_CHANGING_RIGHT,
       */

      if (prepChangeLeftCost < prepKeepLaneCost) {
        cost = prepChangeLeftCost;
        next_state = PREPARE_CHANGING_LEFT;
      } else {
        cost = prepKeepLaneCost;
        next_state = KEEP_LANE;
      }
      /*if (prepChangeRightCost < cost) {
        cost = prepChangeRightCost;
        next_state = PREPARE_CHANGING_RIGHT;
      }*/
    }
      break;

    case PREPARE_CHANGING_LEFT:
      /**
       * TRANSITIONS :
       *   KEEP_LANE
       *   PREPARE_CHANGING_LEFT,
       *   CHANGING_LEFT,
       *   PREPARE_CHANGING_RIGHT,
       */

      if (prepChangeLeftCost < prepKeepLaneCost) {
        cost = prepChangeLeftCost;
        next_state = PREPARE_CHANGING_LEFT;
      } else {
        cost = prepKeepLaneCost;
        next_state = KEEP_LANE;
      }
      /*if (prepChangeRightCost < cost) {
        cost = prepChangeRightCost;
        next_state = PREPARE_CHANGING_RIGHT;
      }
      if (changeLeftCost < cost) {
        cost = prepChangeRightCost;
        next_state = CHANGING_LEFT;
      }*/
      break;

    default:
      current_state = KEEP_LANE;
      break;
  }
}

void DrivingState::setSensorFusion(
    std::vector<std::vector<double>> & sensor_fused) {
  sensor_fusion.clear();
  for (size_t vehicle_id = 0; vehicle_id < sensor_fused.size(); ++vehicle_id) {
    SensorFusion vehicle;
    vector<double> kine = sensor_fused[vehicle_id];
    vehicle.id = kine[0];
    vehicle.x = kine[1];
    vehicle.y = kine[2];
    vehicle.vx = kine[3];
    vehicle.vy = kine[4];
    vehicle.s = kine[5];
    vehicle.d = kine[6];
    sensor_fusion.push_back(vehicle);
  }
}
void DrivingState::setVehicleState(double car_x, double car_y, double car_s,
    double car_d, double car_yaw, double car_speed) {
  this->car_x = car_x;
  this->car_y = car_y;
  this->car_s = car_s;
  this->car_d = car_d;
  this->car_yaw = car_yaw;
  this->car_speed = car_speed;
}

int DrivingState::getVehicleLeadIdx(double lane_d) {
  /* Find leading vehicle */
  double dist_veh_lead = 50;
  int vehicle_lead = -1;

  for (int vehicle_id = 0; vehicle_id < sensor_fusion.size(); ++vehicle_id) {
    SensorFusion kine = sensor_fusion[vehicle_id];
    double kine_s_temp  = kine.s ;
    if (car_s > 6000 && kine_s_temp < 1000) {
      /* Manage wrapping */
      kine_s_temp += road->getMaxS();
    }
    double dist_s = kine_s_temp - car_s;
    double dist_d = kine.d - lane_d;
    if (fabs(dist_d) < 3.0) {
      /* If vehicle in same lane*/
      if (0 < dist_s && dist_s < dist_veh_lead) {
        /* If vehicle in front and closer than any other vehicle below a distance threshold*/
        dist_veh_lead = dist_s;
        vehicle_lead = vehicle_id;
      }
    }
  }
  return vehicle_lead;
}
int DrivingState::getVehicleFollowingIdx(double lane_d) {
  /* Find leading vehicle */
  double dist_veh_lead = 25;
  int vehicle_lead = -1;

  for (int vehicle_id = 0; vehicle_id < sensor_fusion.size(); ++vehicle_id) {
    SensorFusion kine = sensor_fusion[vehicle_id];
    double car_s_temp = car_s;
    if ( kine.s > 6000 && car_s_temp  < 1000) {
      /* Manage wrapping */
      car_s_temp += road->getMaxS();
    }
    double dist_s = car_s_temp - kine.s ;
    double dist_d = kine.d - lane_d;
    if (fabs(dist_d) < 3.0) {
      /* If vehicle in same lane*/
      if (0 < dist_s && dist_s < dist_veh_lead) {
        /* If vehicle in behind and closer than any other vehicle below a distance threshold*/
        dist_veh_lead = dist_s;
        vehicle_lead = vehicle_id;
      }
    }
  }
  return vehicle_lead;
}

double DrivingState::getTargetCarD() const {
  return target_car_d;
}

double DrivingState::getTargetCarSpeed() const {
  return target_car_speed;
}
