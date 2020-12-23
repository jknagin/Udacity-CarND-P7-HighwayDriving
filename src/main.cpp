#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

// for convenience
using nlohmann::json;

int main()
{
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  std::vector<double> map_waypoints_x;
  std::vector<double> map_waypoints_y;
  std::vector<double> map_waypoints_s;
  std::vector<double> map_waypoints_dx;
  std::vector<double> map_waypoints_dy;

  // Waypoint map to read from
  std::string map_file_ = "data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  std::string line;
  while (getline(in_map_, line))
  {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }

  h.onMessage([&map_waypoints_x, &map_waypoints_y, &map_waypoints_s,
               &map_waypoints_dx, &map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {

      auto s = hasData(data);

      if (s != "")
      {
        auto j = json::parse(s);

        std::string event = j[0].get<std::string>();

        if (event == "telemetry")
        {
          // j[1] is the data JSON object

          // start in lane 1
          int lane = 1;

          // reference velocity
          double ref_vel = 49.5; // mph
          
          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = deg2rad(j[1]["yaw"]);
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          ///// BEGIN TRAJECTORY GENERATION
          int prev_size = previous_path_x.size();
          std::vector<double> ptsx, ptsy;
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = car_yaw;

          // If previous path has 0 or 1 points, use current position of car and
          // 1 distance unit behind the car as two points
          if (prev_size < 2)
          {
            double prev_car_x = car_x - std::cos(car_yaw);
            double prev_car_y = car_y - std::sin(car_yaw);
            ptsx.push_back(prev_car_x);
            ptsx.push_back(car_x);
            ptsy.push_back(prev_car_y);
            ptsy.push_back(car_y);
          }
          else // Previous path has at least 2 points
          {
            // Use previous path end point as starting reference
            ref_x = previous_path_x[previous_path_x.size() - 1];
            ref_y = previous_path_y[previous_path_y.size() - 1];
            double ref_x_prev = previous_path_x[previous_path_x.size() - 2];
            double ref_y_prev = previous_path_y[previous_path_y.size() - 2];
            ref_yaw = std::atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);
            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y);
          }

          // In Frenet, add 30m spaced points ahead of starting reference
          std::vector<double> next_waypoint;
          for (int i = 0; i < 3; ++i)
          {
            next_waypoint = getXY(car_s + 30 * (i + 1), 2.0 + 4.0 * lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
            ptsx.push_back(next_waypoint[0]);
            ptsy.push_back(next_waypoint[1]);
          }

          // Shift car reference angle to 0 deg
          double sx, sy;
          for (int i = 0; i < ptsx.size(); ++i)
          {
            sx = ptsx[i] - ref_x;
            sy = ptsy[i] - ref_y;
            ptsx[i] = sx * std::cos(-ref_yaw) - sy * std::sin(-ref_yaw);
            ptsy[i] = sx * std::sin(-ref_yaw) + sy * std::cos(-ref_yaw);
          }

          // Create a spline using ptsx, ptsy
          tk::spline spline;
          spline.set_points(ptsx, ptsy);

          std::vector<double> next_x_vals;
          std::vector<double> next_y_vals;
          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
          
          // Start with previous path points
          for(int i=0;i<previous_path_x.size();++i)
          {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }
          
          // Calculate how to break up spline points to travel at reference velocity
          double horizonX = 30.0;  // meters
          double horizonY = spline(horizonX);
          double distanceToHorizon = std::sqrt(horizonX*horizonX + horizonY*horizonY);
          
          double x_add_on = 0;

          // Fill up rest of path using spline
          for (int i=1;i<= 50-previous_path_x.size();++i)
          {
            double N = distanceToHorizon/(0.02*ref_vel/2.24); // meters
            double x_point = x_add_on + horizonX/N;
            double y_point = spline(x_point);
           
            x_add_on = x_point;

            double x_ref = x_point;
            double y_ref = y_point;
            
            // Rotate back to normal
            x_point = x_ref*std::cos(ref_yaw) - y_ref*std::sin(ref_yaw);
            y_point = x_ref*std::sin(ref_yaw) + y_ref*std::cos(ref_yaw);

            x_point += ref_x;
            y_point += ref_y;

            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }
          ///// END TRAJECTORY GENERATION
  
          json msgJson;
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\"," + msgJson.dump() + "]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        } // end "telemetry" if
      }
      else
      {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    } // end websocket if
  }); // end h.onMessage

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