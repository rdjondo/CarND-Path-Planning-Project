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

using namespace std;

// for convenience
using json = nlohmann::json;


// Transform waypoints in map coordinates to vehicle coordinates.
inline vector<double> mapCoordToVehicleCoordinates(double car_x,
		double car_y, double yaw, double x_way, double y_way) {

	// Fix new coordinate axis origin to vehicle's coordinates
	double x = x_way - car_x;
	double y = y_way - car_y;

	// Limit yaw angle

	// Rotate coordinates to vehicle's coordinates
	double x_veh = x * cos(yaw) - y *sin(yaw);
	double y_veh = x * sin(yaw) + y *cos(yaw) ;
	return {x_veh, y_veh};
}

// Transform waypoints in vehicle coordinates to map coordinates.
inline vector<double> mapPtVehCoordToMapCoordinates(double car_x,
		double car_y, double yaw, double x_way, double y_way){

	// Rotate coordinates to vehicle's coordinates
	double x = x_way * cos(-yaw) - y_way *sin(-yaw);
	double y = x_way * sin(-yaw) + y_way *cos(-yaw) ;

	// Fix new coordinate axis origin to map's coordinates
	double x_map = x + car_x;
	double y_map = y + car_y;

	return {x_map, y_map};
}

int main()
{
    uWS::Hub h;

    // Load up map values for waypoint's x,y,s and d normalized normal vectors
    vector<double> map_waypoints_x;
    vector<double> map_waypoints_y;
    vector<double> map_waypoints_s;
    vector<double> map_waypoints_dx;
	vector<double> map_waypoints_dy;

	// Waypoint map to read from
	load_map(map_waypoints_x, map_waypoints_y, map_waypoints_s,
			map_waypoints_dx, map_waypoints_dy);

	h.onMessage(
			[&map_waypoints_x, &map_waypoints_y, &map_waypoints_s, &map_waypoints_dx, &map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                                                                                                             uWS::OpCode opCode) {
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
                    auto previous_path_x = j[1]["previous_path_x"];
                    auto previous_path_y = j[1]["previous_path_y"];
                    // Previous path's end s and d values
                    double end_path_s = j[1]["end_path_s"];
                    double end_path_d = j[1]["end_path_d"];

                    // Sensor Fusion Data, a list of all other cars on the same side of the road.
                    auto sensor_fusion = j[1]["sensor_fusion"];

                    json msgJson;

                    vector<double> next_x_vals;
                    vector<double> next_y_vals;

                    vector<double> next_s_vals;
                    vector<double> next_d_vals;

                    // DONE: smoothen road path between the waypoints using Spline
                    

                    // TODO: Calculate and control SDC vehicle speed and lane position
                        // Use a Jerk minimizing function to control the vehicle speed (use the two past points for continuity)  
                    // Let's compute the Jerk minimization in the Frenet referential



                    // TODO: Calculate time to collision against other vehicles in Frenet


                    // TODO: Design cost function

                    
                    // TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds

                    double sk = car_s;
                    double sk_dot = car_speed;
                    double sk_double_dot = 0.0;
                    double sk_triple_dot = 0.0;

                    double sT = 100;
                    double sT_dot = 22;
                    double sT_double_dot = 0.0;
                    double sT_triple_dot = 0.0;
                    int N_samples = 50;
                    double delta_t = 0.02;

                    vector< double> start = {sk, sk_dot, sk_double_dot, sk_triple_dot };
                    vector <double> end  = {sT, sT_dot, sT_double_dot, sT_triple_dot };
                    double T = N_samples * delta_t;
                    bool isJerkDefined = true;

                    vector<double> coeffs = JMT(start, end, T, isJerkDefined);
                    if(!isJerkDefined){
                    	end_trajec = coeffs.back();

                    }
                    
                    double dist_inc = 0.4;
                    for(int i = 0; i < N_samples; i++)
                    {
                        double next_s = car_s + (i+1)*dist_inc;
                        double next_d = 6;
                        vector<double> xy_veh = getXY(next_s, next_d, map_waypoints_s, map_waypoints_x,  map_waypoints_y);
                        vector<double> xy = mapCoordToVehicleCoordinates(car_x, car_y, car_yaw, xy_veh[0], xy_veh[1]);

                        xy = mapPtVehCoordToMapCoordinates(car_x, car_y, car_yaw, xy[0], xy[1]);
                        next_x_vals.push_back(xy[0]);
                        next_y_vals.push_back(xy[1]);
                        next_s_vals.push_back(next_s);
                        next_d_vals.push_back(next_d);
                    }

                    msgJson["next_x"] = next_x_vals;
                    msgJson["next_y"] = next_y_vals;

                    auto msg = "42[\"control\"," + msgJson.dump() + "]";

                    //this_thread::sleep_for(chrono::milliseconds(1000));
                    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                    // std::cout<< "DOWN:" << j<<std::endl;
                    // std::cout<< "UP:" << msg<<std::endl;
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
                       size_t, size_t) {
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

    h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
        std::cout << "Connected!!!" << std::endl;
    });

    h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                           char *message, size_t length) {
        ws.close();
        std::cout << "Disconnected" << std::endl;
    });

    int port = 4567;
    if (h.listen(port))
    {
        std::cout << "Listening to port " << port << std::endl;
    }
    else
    {
        std::cerr << "Failed to listen to port" << std::endl;
        return -1;
    }
    h.run();
}
