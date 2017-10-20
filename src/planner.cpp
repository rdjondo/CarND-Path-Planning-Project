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
#include "planner.h"

using namespace std;

static const double MAX_SPEED = 20.4;

static const double SMALL_VALUE = 1e-3;
static const double LARGE_VALUE = 1e6;
static const double VERY_LARGE_VALUE = 1e9;

DrivingState::DrivingState(double road_max_s_value) {
  sensor_fusion.clear();
  road_max_s =  road_max_s_value;
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


// Calculate time and distance to collision against other vehicles in vicinity
double DrivingState::getTimeToCollision(const size_t vehicle_idx){
  double time_to_collision = LARGE_VALUE;
  if (vehicle_idx >= 0) {
    SensorFusion veh = sensor_fusion[vehicle_idx];
    double dist = veh.s - car_s;
    double delta_speed = car_speed/2.23694 - sqrt(veh.vx * veh.vx + veh.vy * veh.vy);
    time_to_collision = (dist) / (SMALL_VALUE + delta_speed);

    const double FORBIDDEN_DIST_MARGIN = 15.0; // 15.0 metres is the acceptable limit
    const double FORBIDDEN_TIME_MARGIN = 15.0; // 15 secs is the acceptable limit
    if(fabs(dist)<FORBIDDEN_DIST_MARGIN){
      /* distance is too short : make time to collision  small */
      time_to_collision = SMALL_VALUE;
    } else if(time_to_collision < 0){
      time_to_collision = LARGE_VALUE;
    }else if(time_to_collision<FORBIDDEN_TIME_MARGIN){
      /* time_to_collision is too short : make time to collision even smaller */
      time_to_collision = SMALL_VALUE;
    }
  }
  return time_to_collision;
}

// Cost function for staying in current lane
double DrivingState::getKeepLaneCost() {
  int vehicle_lead = getVehicleLeadIdx(car_d);
  double time_to_collision = getTimeToCollision(vehicle_lead);

  cout << "KeepLane time: " << time_to_collision << "  ";
  double cost = min(LARGE_VALUE, 1 / (SMALL_VALUE + time_to_collision));
  return cost;
}

// Cost function for changing lane
double DrivingState::getPrepareChangingLaneCost(const double lane_d) {
  double time_to_collision = LARGE_VALUE;

  int vehicle_lead = getVehicleLeadIdx(target_car_d + lane_d);
  int vehicle_follow = getVehicleFollowingIdx(target_car_d + lane_d);

  time_to_collision = getTimeToCollision(vehicle_lead);
  cout << "Lead time " << lane_d << ": " << time_to_collision << "  ";
  double cost = 1 / (SMALL_VALUE + time_to_collision);

  time_to_collision = getTimeToCollision(vehicle_follow);
  cout << "Foll time " << lane_d << ": " << time_to_collision << "  ";
  cost += 1 / (SMALL_VALUE + time_to_collision);

  cost = min(LARGE_VALUE, cost);
  return cost;
}

double DrivingState::getChangingLeftCost() {
  const double lane_d = -4.0;
  // Very large cost for changing to a lane that would be out of the road
  if (target_car_d + lane_d <= 0) {
    return VERY_LARGE_VALUE;
  } else {
    return getPrepareChangingLaneCost(lane_d);
  }
}

double DrivingState::getChangingRightCost() {
  const double lane_d = 4.0;
  // Very large cost for changing to a lane that would be out of the road 
  if (target_car_d + lane_d >= 12) {
    return VERY_LARGE_VALUE;
  } else {
    return getPrepareChangingLaneCost(lane_d);
  }
}

void DrivingState::adaptativeCruiseControl() {
  /* By default, use Max speed */
  target_car_speed = MAX_SPEED;

  int vehicle_lead = getVehicleLeadIdx(car_d);
  /* Assign its speed as target speed */
  SensorFusion veh = sensor_fusion[vehicle_lead];
  if (vehicle_lead >= 0) {
    double delta_s = veh.s - car_s;
    const double braking_margin = 25.0;
    const double slowing_margin = 45.0;
    double leading_car_speed = sqrt(veh.vx * veh.vx + veh.vy * veh.vy);

    if (delta_s < braking_margin) {
      // saturate the speed to MAX_SPEED for safety
      double gain = 2.5;
      double speed_adjustment = gain * (braking_margin - delta_s);
      target_car_speed = max(0.0,
          min(leading_car_speed, leading_car_speed - speed_adjustment));
    } else if (delta_s < slowing_margin) {
      double gain = 1.01;
      // saturate the speed to MAX_SPEED for safety
      target_car_speed = max(0.0, min(MAX_SPEED, leading_car_speed * gain));
    }
  }
}

void DrivingState::nextState() {

  /* Manage vehicle speed*/
  adaptativeCruiseControl();

  /* Calculate costs for staying in current lane or changing lane */
  double keepLaneCost = getKeepLaneCost();
  cout<<endl;
  double changeLeftCost = getChangingLeftCost();
  cout<<endl;
  double changeRightCost = getChangingRightCost();

  cout<<endl;

  cout << "KL cost:" << keepLaneCost << "  ";
  cout << "PL cost:" << changeLeftCost << "  ";
  cout << "PR cost:" << changeRightCost << endl;

  double cost;

  switch (current_state) {
    case KEEP_LANE: {
      /* TARGETS
       * 2 possible targets:  speed = max speed or speed = matching speed of vehicle in front
       *
       * TRANSITIONS :
       *   KEEP_LANE
       *   CHANGING_LEFT,
       *   CHANGING_RIGHT,
       */

      /* Find minimal cost to calculate next state*/
      double temp_target_car_d = target_car_d;
      if (changeLeftCost > keepLaneCost) { /* Priority is to keep in lane */
        cost = keepLaneCost;
        next_state = KEEP_LANE;
      } else {
        cost = changeLeftCost;
        next_state = CHANGING_LEFT;
        temp_target_car_d = target_car_d - 4.0;
      }

      if (changeRightCost < cost) {
       next_state = CHANGING_RIGHT;
       temp_target_car_d = target_car_d + 4.0;
       }

      target_car_d = temp_target_car_d;
    }
      break;

    case CHANGING_LEFT:
      /**
       * TRANSITIONS :
       *   KEEP_LANE
       */
      if (fabs(target_car_d - car_d) < 0.2) {
        next_state = KEEP_LANE;
      }
      break;

    case CHANGING_RIGHT:
      /**
       * TRANSITIONS :
       *   KEEP_LANE
       */
      if (fabs(target_car_d - car_d) < 0.2) {
        next_state = KEEP_LANE;
      }
      break;

    default:
      current_state = KEEP_LANE;
      break;
  }
  current_state = next_state;
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

/* TODO: move the constant inside the class*/
static const double DIST_VEH_LEAD = 75;

// Get the id for the closest following vehicle  
//TODO: Merge common parts from getVehicleLeadIdx and getVehicleFollowingIdx 
int DrivingState::getVehicleLeadIdx(const double lane_d) {
  /* Find leading vehicle */
  double dist_veh_lead = DIST_VEH_LEAD;
  int vehicle_lead = -1;

  for (int vehicle_id = 0; vehicle_id < sensor_fusion.size(); ++vehicle_id) {
    SensorFusion kine = sensor_fusion[vehicle_id];
    double kine_s_temp = kine.s;
    if (car_s > 6000 && kine_s_temp < 1000) {
      /* Manage wrapping */
      kine_s_temp += road_max_s;
    }
    double dist_s = kine_s_temp - car_s;
    double dist_d = kine.d - lane_d;
    if (fabs(dist_d) < 3.5) {
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

/* TODO: move the constant inside the class*/
static const double DIST_VEH_LEAD_FOLLOW = 100;

// Get the id for the closest following vehicle  
int DrivingState::getVehicleFollowingIdx(const double lane_d) {
  /* Find leading vehicle */
  double dist_veh_lead = DIST_VEH_LEAD_FOLLOW;
  int vehicle_lead = -1;

  for (int vehicle_id = 0; vehicle_id < sensor_fusion.size(); ++vehicle_id) {
    SensorFusion kine = sensor_fusion[vehicle_id];
    double car_s_temp = car_s;
    if (kine.s > 6000 && car_s_temp < 1000) {
      /* Manage wrapping */
      car_s_temp += road_max_s;
    }
    double dist_s = car_s_temp - kine.s;
    double dist_d = kine.d - lane_d;
    if (fabs(dist_d) < 2.0) {
      /* If vehicle in same lane*/
      if (0 <= dist_s && dist_s < dist_veh_lead) {
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
