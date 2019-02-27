#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"

#include <algorithm> //added by me, to use max(), min()
#include "spline.h"  //downloaded spline.h file

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

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

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy]
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

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
          //== simple test, move vehicle forward
//          double dist_inc = 0.5;
//          for (int i = 0; i < 50; ++i) {
//            next_x_vals.push_back(car_x+(dist_inc*i)*cos(deg2rad(car_yaw)));
//            next_y_vals.push_back(car_y+(dist_inc*i)*sin(deg2rad(car_yaw)));
//          }
          double pos_x;
          double pos_y;
          double angle;
          int path_size = std::min(previous_path_x.size(), 5); //only keep up to 5 previous history points
          for(int i = 0; i < path_size; ++i) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          if (path_size == 0) {
            pos_x = car_x;
            pos_y = car_y;
            angle = deg2rad(car_yaw);
          } else {
            pos_x = previous_path_x[path_size-1];
            pos_y = previous_path_y[path_size-1];

            double pos_x2 = previous_path_x[path_size-2];
            double pos_y2 = previous_path_y[path_size-2];
            angle = atan2(pos_y-pos_y2,pos_x-pos_x2);
          }

          //to do (follow the car before you):
          //    1. find map waypoints after current position
          //    2. find current position s and d
          //    3. propose path that keeps current d but advances s
          //    4. if path intersect with any vehicles around?
          //int nextPt = NextWaypoint(pos_x, pos_y, angle, map_waypoints_x, map_waypoints_y);
          double pos_s;
          double pos_d;
          vector<double> sd = getFrenet(pos_x, pos_y, angle, map_waypoints_x, map_waypoints_y);
          pos_s = sd[0];
          pos_d = sd[1];

          std::cout<<"current pos_x, pos_y: "<<pos_x<<" "<<pos_y<<std::endl;
          std::cout<<"current pos_s: "<<pos_s<<std::endl;

          double ref_vel = 49.5; //mph
          double delta_T = 0.02; //0.02 second per path point
          double lane = 1; //lane = 0, 1, 2
          double s_inc= ref_vel*1600.0/3600.0*deta_T; // 50mph -> 22.352 m/s -> 0.447 m/0.02s
          double target_s;
          double target_d;
          int nPts = 50-path_size;  // append n points from (pos_s, pos_d) to (target_s, target_d)
//          target_s = pos_s+nPts*s_inc;
//          target_d = pos_d; //stay in lane

          //check other vehicles
          //std::cout<<"set target_s to "<<target_s<<std::endl;

          for(int i= 0; i< sensor_fusion.size(); i++)
          {
            //std::cout<<sensor_fusion[i]<<", ";
        	  auto obj = sensor_fusion[i];
//        	  if(obj["s"]<target_s && obj["s"]>=pos_s && fabs(obj["d"]-target_d)<2)
//        		  target_s = obj["s"];
        	  double obj_s = obj[5];
        	  double obj_d = obj[6];

        	  std::cout<<" obj_s = "<<obj_s<<std::endl;

        	  if(obj_s<target_s && obj_s>=pos_s && fabs(obj_d-target_d)<2)
        	  {
        	  	  target_s = obj_s;
        	  	  std::cout<<"!! Update target_s to "<<target_s<<std::endl;
        	  }
          }

//          // Generate trajectory based on (pos_s, pos_d) and (obj_s, obj_d) using polynomial
//          vector<double> s_coeff = JMT(s_start, s_end);
//          vector<double> d_coeff = JMT(d_start, d_end, )

          //generate trajectory based on spline function
          vector<double> ptsx;
          vector<double> ptsy;
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);
          //generate points to make sure path is tangent to current yaw
          if(prev_size<2) //if previous size is almost empty, use the car as starting reference
          {
        	  double prev_car_x = car_x-cos(car_yaw);
        	  double prev_car_y = car_y-sin(car_yaw);
        	  ptsx.push_back(pre_car_x);
        	  ptsx.push_back(car_x);

        	  ptsy.push_back(prev_car_y);
        	  ptsy.push_back(car_y);

          }
          else
          {
        	  ref_x = previous_path_x[prev_size-1];
        	  ref_y = previous_path_y[prev_size-1];
        	  double ref_x_prev = previous_path_x[prev_size-2];
        	  double ref_y_prev = previous_path_y[prev_size-2];
        	  ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);

        	  ptsx.push_back(ref_x_prev);
        	  ptsx.push_back(ref_x);

        	  ptsy.push_back(ref_y_prev);
        	  ptsy.push_back(ref_y);
          }

          vector<double> next_wp0 = getXY(car_s+30, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s+60, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s+90, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);

          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);

          for(int i = 0; i<ptsx.size(); i++)
          {
        	  // shift car reference angle to 0 degrees
        	  double shift_x = ptsx[i]-ref_x;
        	  double shift_y = ptsy[i]-ref_y;
        	  ptsx[i] = (shift_x*cos(0-ref_yaw)-shift_y*sin(0-ref_yaw));
        	  ptsy[i] = (shift_x*sin(0-ref_yaw)+shift_y*cos(0-ref_yaw));
          }
          //create spline
          tk::spline s;
          s.set_points(ptsx, ptsy);
          vector<double> next_x_vals;
          vector<double> next_y_vals;
          for(int i=0;i<previous_path_x.size(); i++)
          {
        	  next_x_vals.push_back(previous_path_x[i]);
        	  next_y_vals.push_back(previous_path_y[i]);
          }
          double target_x = 30.0;
          double target_y = s(target_x);
          double target_dist = sqrt(target_x*target_x+target_y*target_y);
          double x_add_on = 0;
          // fill up the rest of path planner
          for(int i=1; i<=50-previous_path_x.size(); i++)
          {
        	  double N = (target_dist/(.02*ref_vel/2.24));
        	  double x_point = x_add_on+(target_x)/N;
        	  double y_point = s(x_point);
        	  x_add_on = x_point;
        	  double x_ref = x_point;
        	  double y_ref = y_point;
        	  x_point = (x-ref*cos(ref_yaw)-y_ref*sin(ref_yaw));
        	  y_point = (y_ref*sin(ref_yaw)+y_ref*cos(ref_yaw));
        	  x_point += ref_x;
        	  next_x_vals.push_back(x_point);
        	  next_y_vals.push_back(y_point);
          }
          s_inc = (target_s-pos_s)/nPts;
          for (int i = 0; i < nPts; ++i) {
        	pos_s += s_inc;
        	vector<double> xy = getXY(pos_s, pos_d, map_waypoints_s,  map_waypoints_x, map_waypoints_y);
            next_x_vals.push_back(xy[0]);
            next_y_vals.push_back(xy[1]);
          }
         // show sensor fusion data: [id, x, y, vx, vy, s, d]
//          std::cout<<"sensor_fusion data: ";
//          for(int i= 0; i< sensor_fusion.size(); i++)
//          {
//        	  std::cout<<sensor_fusion[i]<<", ";
//          }

          //std::cout<<"target pos_s: "<<pos_s<<std::endl;
//          std::cout<<"Planned Path:===== ";
//          for(int i=0;i<next_x_vals.size();i++)
//        	  std::cout<<next_x_vals[i]<<" "<<next_y_vals[i]<<std::endl;


          //===================original Udacity code below==========
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
