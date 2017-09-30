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

// The max s value before wrapping around the track back to 0
double max_s = 6945.554;

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

                    vector<double> smooth_road_x;
                    vector<double> smooth_road_y;

                    // TODO: smoothen road path between the waypoints using Spline
                    double s_inc = 0.5;

                    static tk::spline spline_func;
                    vector<double>  to_interpolate_x;
                    vector<double>  to_interpolate_y;
                    to_interpolate_x.clear();
                    to_interpolate_y.clear();
                    
                    std::cout<<" X interp: ";
                    car_yaw = car_yaw/180.0*pi();
                    //car_yaw = fmod(car_yaw+2*pi(), pi());
                    double old_x = -10000.0;
                    for(int i = 1; i < 50; i++)
                    {
                        double next_s = car_s + i*s_inc;
                        double next_d = 6;
                        vector<double> xy = getXY(next_s, next_d, map_waypoints_s, map_waypoints_x,  map_waypoints_y);
                        xy = mapCoordToVehicleCoordinates(car_x, car_y, car_yaw, xy[0], xy[1]);
                        if(old_x<xy[0]){
                        	to_interpolate_x.push_back(xy[0]);
                        	to_interpolate_y.push_back(xy[1]);
                        }
                        std::cout<<xy[0]<<", ";
                        old_x = xy[0];
                    }
                    std::cout<<std::endl;

                    bool reverse_sign = false;

                    for (int i = 0; i < to_interpolate_x.size()-1; ++i) {
                    	if(to_interpolate_x[i]>to_interpolate_x[i+1]){
                    		reverse_sign = true;
                    		std::cout<<"OOOPS, "<<"car_yaw="<<car_yaw<<"; to_interpolate_x:"<<
                    				to_interpolate_x[i]<<" is greater than "<<
                    				to_interpolate_x[i+1]<<std::endl;
                    		break;
                    	}

					}

                    if(reverse_sign){
                    	for(int i=0; i < to_interpolate_x.size();++i){
                    		to_interpolate_x[i] = -to_interpolate_x[i];
                    	}
                    }

                    spline_func.set_points(to_interpolate_x,to_interpolate_y);    // Calculate interpolated function


                    // TODO: Calculate and control SDC vehicle speed and lane position
                        // Use a Jerk minimizing function to control the vehicle speed (use the two past points for continuity)  
                    // TODO: Calculate time to collision against other vehicles in Frenet
                    // TODO: Design cost function

                    
                    // TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
                    
                    double dist_inc = 0.4;
                    for(int i = 0; i < 60; i++)
                    {
                        double next_s = car_s + (i+1)*dist_inc;
                        double next_d = 6;
                        vector<double> xy_veh = getXY(next_s, next_d, map_waypoints_s, map_waypoints_x,  map_waypoints_y);
                        vector<double> xy = mapCoordToVehicleCoordinates(car_x, car_y, car_yaw, xy_veh[0], xy_veh[1]);
                        double y = spline_func(xy[0]);
                        xy = mapPtVehCoordToMapCoordinates(car_x, car_y, car_yaw, xy[0], y);
                        next_x_vals.push_back(xy[0]);
                        next_y_vals.push_back(xy[1]);
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
