#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"
#include "utils.h"
#include "jmt.h"
#include "optim_jmt.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// Transform waypoints in map coordinates to vehicle coordinates.
inline vector<double> mapCoordToVehicleCoordinates(double car_x, double car_y,
		double yaw, double x_way, double y_way) {

	// Fix new coordinate axis origin to vehicle's coordinates
	double x = x_way - car_x;
	double y = y_way - car_y;

	// Limit yaw angle

	// Rotate coordinates to vehicle's coordinates
	double x_veh = x * cos(yaw) - y * sin(yaw);
	double y_veh = x * sin(yaw) + y * cos(yaw);
	return
	{	x_veh, y_veh};
}

// Transform waypoints in vehicle coordinates to map coordinates.
inline vector<double> mapPtVehCoordToMapCoordinates(double car_x, double car_y,
		double yaw, double x_way, double y_way) {

	// Rotate coordinates to vehicle's coordinates
	double x = x_way * cos(-yaw) - y_way * sin(-yaw);
	double y = x_way * sin(-yaw) + y_way * cos(-yaw);

	// Fix new coordinate axis origin to map's coordinates
	double x_map = x + car_x;
	double y_map = y + car_y;

	return
	{	x_map, y_map};
}

int main() {
	uWS::Hub h;

	// Load up map values for waypoint's x,y,s and d normalized normal vectors
	vector<double> map_waypoints_x;
	vector<double> map_waypoints_y;
	vector<double> map_waypoints_s;
	vector<double> map_waypoints_dx;
	vector<double> map_waypoints_dy;

	// Waypoint map to read from
	load_map(map_waypoints_x, map_waypoints_y, map_waypoints_s, map_waypoints_dx,
			map_waypoints_dy);

	h.onMessage(
			[&map_waypoints_x, &map_waypoints_y, &map_waypoints_s, &map_waypoints_dx, &map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
					uWS::OpCode opCode)
			{
				// "42" at the start of the message means there's a websocket message event.
				// The 4 signifies a websocket message
				// The 2 signifies a websocket event
				//auto sdata = string(data).substr(0, length);
				//cout << sdata << endl;
				if (length && length > 2 && data[0] == '4' && data[1] == '2')
				{

					auto s = hasData(data);

					if (s != "")
					{
						auto j = json::parse(s);

						string event = j[0].get<string>();

						if (event == "telemetry")
						{
							// j[1] is the data JSON object

							// Main car's localization Data
							double car_x = j[1]["x"];
							double car_y = j[1]["y"];
							double car_s = j[1]["s"];
							double car_d = j[1]["d"];
							double car_yaw = j[1]["yaw"];
							double car_speed = j[1]["speed"];

							// Previous path data given to the Planner
							json previous_path_x = j[1]["previous_path_x"];
							json previous_path_y = j[1]["previous_path_y"];
							// Previous path's end s and d values
							double end_path_s = j[1]["end_path_s"];
							double end_path_d = j[1]["end_path_d"];

							// Sensor Fusion Data, a list of all other cars on the same side of the road.
							auto sensor_fusion = j[1]["sensor_fusion"];

							json msgJson;


							vector<double> next_s_vals;
							vector<double> next_d_vals;

							// DONE: smoothen road path between the waypoints using Spline

							// TODO: Calculate and control SDC vehicle speed and lane position
							// Use a Jerk minimizing function to control the vehicle speed (use the two past points for continuity)
							// Let's compute the Jerk minimization in the Frenet referential

							// TODO: Calculate time to collision against other vehicles in Frenet

							// TODO: Design cost function

							// TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds

							const double delta_t = 0.02;
							static vector<double> coeffs_old = {0.0, 0.0 , 0.0 , 0.0 , 0.0 , 0.0};
							static int N_samples_old = -1;

							static double last_s = 1e20; /* initialise to an arbitrary large number*/
							const int N_samples = 100; //(int) T_optimised/delta_t;
							const int previous_path_size = previous_path_x.size();
							double sk = car_s;
							double sk_dot = car_speed/2.23694;
							double sk_double_dot = 0.0;

							double sT_dot = 18;
							//if(((int)floor(car_s/100))%2==0) sT_dot = 20;
							double sT_double_dot = 0.0;

							/* Find delay single last plan */
							double delay_t=0.24;
							if(N_samples_old>=0) {
								double t_incr=0.1;
								int old_dir = 1;
								int dir = 1;
								double s_estim = polyval(coeffs_old, delay_t);
								while(fabs(s_estim-car_s)>0.01 && delay_t>-0.2 ){
									if(s_estim>car_s){
										delay_t -= t_incr;
										dir = 1;
									} else{
										delay_t += t_incr;
										dir = -1;
									}
									s_estim = polyval(coeffs_old, delay_t);
									if(dir != old_dir){
										t_incr/=2.0;
									}
									old_dir = dir;
								}
								if(delay_t<0.0) delay_t = 0.0;

								/* Display estimated telemetry from old plan */
								double last_t = (N_samples-1)*delta_t;
								car_s = last_s;
								double sk = last_s;
								cout<<"last_s:"<<last_s<<"   last_t:"<<last_s<<endl;
								cout<<"len(previous_path_x):"<<previous_path_size<<endl;
								cout<<"car_s:"<<car_s<<"  estim car_s:"<<polyval(coeffs_old, last_t)<<endl;
								vector<double> speed_poly = polyder(coeffs_old);
								sk_dot = polyval( speed_poly, last_t);
								cout<<"car_speed:"<<car_speed<<"  estim car_speed:"<<sk_dot*2.23694<<endl;
								vector<double> acc_poly = polyder(speed_poly);
								sk_double_dot = polyval( acc_poly, last_t);
								cout<<"estim car_acc:"<<sk_double_dot<<endl;
								/* Let's compute the kinematic parameters on the last point */
								cout<<"Previous x path"<<previous_path_x<<" size "<<previous_path_size<<endl;
								//cout<<"Previous y path"<<previous_path_y<<" size "<<previous_path_size<<endl;
							}


							double sT_optimised = 0.0;
							double T_optimised = 0.0;

							vector<double> coeffs = optim_jmt(sk, sk_dot, sk_double_dot, sT_dot,
									sT_double_dot, sT_optimised, T_optimised);

							coeffs_old = coeffs;

							N_samples_old = N_samples;

							cout<<"coeffs = ["<<coeffs[0]<<", "<<coeffs[1]<<", "<<coeffs[2]<<", "<<coeffs[3]
							<<", "<<coeffs[4]<<", "<<coeffs[5]<<"] "<<endl;


							static vector<double> next_x_vals(N_samples);
							static vector<double> next_y_vals(N_samples);
							next_x_vals.clear();
							next_y_vals.clear();
							int start_copy = (int)(delay_t/delta_t)+1;
							for(int i = start_copy; i< previous_path_size - 1 && N_samples_old>=0; ++i) {
								double x = previous_path_x[i];
								double y = previous_path_y[i];
								next_x_vals.push_back(x);
								next_y_vals.push_back(y);
							}
							double old_x = next_x_vals.back();
							double old_y = next_y_vals.back();

							// double dist_inc = 0.4;
							double next_s;
							int t_idx = 0;
							for(int i = previous_path_size+1; i < N_samples; ++i, t_idx++;)
							{
								next_s = polyval(coeffs, (t_idx)*delta_t);


								double next_d = 6;
								vector<double> xy_veh = getXY(next_s, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
								//vector<double> xy = mapCoordToVehicleCoordinates(car_x, car_y, car_yaw, xy_veh[0], xy_veh[1]);
								//xy = mapPtVehCoordToMapCoordinates(car_x, car_y, car_yaw, xy[0], xy[1]);
								next_x_vals.push_back(xy_veh[0]);
								next_y_vals.push_back(xy_veh[1]);
							}
							last_s = next_s;

							msgJson["next_x"] = next_x_vals;
							msgJson["next_y"] = next_y_vals;

							auto msg = "42[\"control\"," + msgJson.dump() + "]";

							ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
							// std::cout<< "DOWN:" << j<<std::endl;
							// std::cout<< "UP:" << msg<<std::endl;
							this_thread::sleep_for(chrono::milliseconds(400));
						}
					}
					else
					{
						// Manual driving
						std::string msg = "42[\"manual\",{}]";
						ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
					}
				}
			});

	// We don't need this since we're not using HTTP but if it's removed the
	// program
	// doesn't compile :-(
	h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
			size_t, size_t)
	{
		const std::string s = "<h1>Hello world!</h1>";
		if (req.getUrl().valueLength == 1)
		{
			res->end(s.data(), s.length());
		}
		else
		{
			// i guess this should be done more gracefully?
			res->end(nullptr, 0);
		}
	});

	h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req)
	{
		std::cout << "Connected!!!" << std::endl;
	});

	h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
			char *message, size_t length)
	{
		ws.close();
		std::cout << "Disconnected" << std::endl;
	});

	int port = 4567;
	if (h.listen(port)) {
		std::cout << "Listening to port " << port << std::endl;
	} else {
		std::cerr << "Failed to listen to port" << std::endl;
		return -1;
	}
	h.run();
}
