/*
 * planner.h
 *
 *  Created on: 17 Oct 2017
 *      Author: rdjondo
 */

#ifndef PLANNER_H_
#define PLANNER_H_



struct SensorFusion {
  int id;
  double x;
  double y;
  double vx;
  double vy;
  double s;
  double d;
};

/**
 * The class defines the behaviour of the vehicle on the road
 * The vehicle balances speed targets and steering costs.
 */
class DrivingState {
public:
  enum State {
    KEEP_LANE = 0,
    PREPARE_CHANGING_LEFT,
    PREPARE_CHANGING_RIGHT,
    CHANGING_LEFT,
    CHANGING_RIGHT,
  };

  DrivingState(RoadGeometry & road_init);
  virtual ~DrivingState();

  void nextState();

  void setSensorFusion(std::vector<std::vector<double>> & sensor_fused);
  void setVehicleState(double car_x, double car_y, double car_s, double car_d,
      double car_yaw, double car_speed);
  double getTargetCarD() const;
  double getTargetCarSpeed() const;

private:
  std::vector<SensorFusion> sensor_fusion;
  RoadGeometry * road;
  State current_state;
  State next_state;
  double car_x;
  double car_y;
  double car_s;
  double car_d;
  double car_yaw;
  double car_speed;
  double target_car_speed;
  double target_car_d;

  double getKeepLaneCost();
  double getPrepareChangingLeftCost();
  double getPrepareChangingRightCost();
  double getChangingRightCost();
  double getChangingLeftCost();
  int getVehicleLeadIdx(double lane_d);
  int getVehicleFollowingIdx(double lane_d);
};



#endif /* PLANNER_H_ */
