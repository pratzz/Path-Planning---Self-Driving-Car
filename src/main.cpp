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
using std::string;
using std::vector;

bool CheckIfCarIsClose(json sensor_fusion, int lane, double car_s, int prev_size, bool check_behind)
{
  for(int i=0;i<sensor_fusion.size();i++)
  {
    float check_car_d = sensor_fusion[i][6];
    if(check_car_d < (2+4*lane+2) && check_car_d > (2+lane*4-2)) //check if car is in the specified lane
    {
      double vx = sensor_fusion[i][3];
      double vy = sensor_fusion[i][4];
      double check_speed = sqrt(vx*vx +vy*vy);
      double check_car_s = sensor_fusion[i][5];
      
      check_car_s += ((double)prev_size*0.02*check_speed); //estimate the car's s position
      
      if(((check_car_s > car_s) && ((check_car_s - car_s) < 30)) || //if car is ahead and within 30m
         (check_behind && (check_car_s < car_s) && ((car_s - check_car_s <15)))) //if car is behind and within 15m
        return true;
    }
  }
  return false;
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
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
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
  //starting in lane 1
  int lane = 1;
  
  //the reference increment in speed
  double ref_vel = 0;

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy,&lane,&ref_vel]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
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

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;

          int prev_size = previous_path_x.size();
          
          if(prev_size > 0)
          {
            car_s = end_path_s;
          }
          
          bool car_ahead = CheckIfCarIsClose(sensor_fusion, lane, car_s, prev_size, false); //check if there is a car ahead
          bool car_on_left = false;
          bool car_on_right = false;
          if(car_ahead)
          {
            car_on_left = CheckIfCarIsClose(sensor_fusion, lane-1, car_s, prev_size, true); //check if there is a car ahead or behind on the left lane
            car_on_right = CheckIfCarIsClose(sensor_fusion, lane+1, car_s, prev_size, true); //check if there is a car ahead or behind on the right lane
            
            if(!car_on_left && lane>0) //if there is a left lane and there is no car on the left lane
              lane--;
            else if(!car_on_right && lane!=2) //if there is a right lane and there is no car on the right lane
              lane++;
            else //keep lane and slow down
              ref_vel -= 0.224;
          }
          else
          {
            if(ref_vel<49.5) //road is free, accelerate
              ref_vel += 0.224;
          }
          /*
          for(int i=0; i<sensor_fusion.size(); i++)
          {
            float check_car_d = sensor_fussion[i][6];
            if(check_car_d < (2+4*lane+2) && check_car_d > (2+4*lane-2))
            {
              double vx = sensor_fusion[i][3];
              double vy = sensor_fusion[i][4];
              double check_speed = sqrt(vx*vx+vy*vy);
              double check_car_s = sensor_fusion[i][5];
              
              check_car_s += (double)prev_size*0.02*check_speed;

              if (check_car_s>car_s && (check_car_s-car_s)<30)
              {
//                 ref_vel = 29.5;
                car_ahead = true;
              }
              
            }
          }
          */
          
//           if(too_close)
//              ref_vel -= 0.224;
//           else if(ref<49.5)
//             ref_vel += 0.224;
                    
          vector<double> ptsx;
          vector<double> ptsy;
          
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);
          
          //No previous points
          if(prev_size < 2)
          {
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);
            
            ptsx.push_back(prev_car_x);
            ptsy.push_back(prev_car_y);
            
            ptsx.push_back(car_x);            
            ptsy.push_back(car_y);
          }
          else   //Add last 2 previous points for smoother ride
          {
            
            ref_x = previous_path_x[prev_size-1];
            ref_y = previous_path_y[prev_size-1];
              
            double ref_x_prev = previous_path_x[prev_size-2];
            double ref_y_prev = previous_path_y[prev_size-2];
            
            ptsx.push_back(ref_x_prev);
            ptsy.push_back(ref_y_prev);
            
            ptsx.push_back(ref_x);            
            ptsy.push_back(ref_y);
            
            ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);
          }
          
          //Add 3 30m spaced points
          vector<double> next_waypt_30 = getXY(car_s+30,(2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_waypt_60 = getXY(car_s+60,(2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_waypt_90 = getXY(car_s+90,(2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          
          ptsx.push_back(next_waypt_30[0]);
          ptsy.push_back(next_waypt_30[1]);
          
          ptsx.push_back(next_waypt_60[0]);
          ptsy.push_back(next_waypt_60[1]);
          
          ptsx.push_back(next_waypt_90[0]);          
          ptsy.push_back(next_waypt_90[1]);
          
          // transforming from global coordinates to vehicle coordinates, making the angle zero for simple calculations
          for(int i =0; i<ptsx.size(); i++)
          {
            double shift_x = ptsx[i]-ref_x;
            double shift_y = ptsy[i]-ref_y;
            
            ptsx[i] = (shift_x * cos(0-ref_yaw) - shift_y * sin(0-ref_yaw));
            ptsy[i] = (shift_x * sin(0-ref_yaw) + shift_y * cos(0-ref_yaw));
          }
          
          vector<double> next_x_vals;
          vector<double> next_y_vals;
          
          //create spline
          tk::spline s;
          
          //set the 5 points in spline
          s.set_points(ptsx, ptsy);
          
          for(int i=0; i<previous_path_x.size(); i++)
          {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }
          
          //break up spline points to get equal spaced points to maintain speed
          double target_x = 30.0;
          double target_y = s(target_x);
          double target_dist = sqrt((target_x * target_x) + (target_y * target_y));
          
          double x_add_on = 0;
          
          for (int i=0; i<50-previous_path_x.size(); i++)
          {
            double N = target_dist/(0.02 * ref_vel / 2.24);
            double x_point = x_add_on + target_x / N;
            double y_point = s(x_point);
            
            x_add_on = x_point;
            
            double x_ref = x_point;
            double y_ref = y_point;
            
            x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
            y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);
            
            x_point += ref_x;
            y_point += ref_y;
            
            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }
          
          /*
          // Trial 1 - Get the car started by driving straight at 49.5 MPH
          double dist_inc = 0.5;
          for (int i = 0; i < 50; ++i) {
            next_x_vals.push_back(car_x+(dist_inc*i)*cos(deg2rad(car_yaw)));
            next_y_vals.push_back(car_y+(dist_inc*i)*sin(deg2rad(car_yaw)));
          }
          */
          /*
          //Trial 2 - Try using frenet points to drive straight in the lane
          double dist_inc = 0.5;
          double next_s;
          double next_d = 6; // We want to stay in the current lane
          vector<double>  xy;
          for (int i = 0; i < 50; ++i) {
            next_s = car_s+(i+1)*dist_inc;
            
            // Get x,y coordinates from frenet coordinates
            xy = getXY(next_s, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
            
            next_x_vals.push_back(xy[0]);
            next_y_vals.push_back(xy[1]);
          }
          */
          
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
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
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  
  h.run();
}