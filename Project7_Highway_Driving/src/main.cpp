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
enum state {keep_lane, move_left, move_right};

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
          double pos_x;
          double pos_y;
          double angle;
          int path_size = previous_path_x.size(); //only keep up to 5 previous history points
          if(path_size>5)
        	  path_size = 5;
//          for(int i = 0; i < path_size; ++i) {
//            next_x_vals.push_back(previous_path_x[i]);
//            next_y_vals.push_back(previous_path_y[i]);
//          }
//
//          if (path_size == 0) {
//            pos_x = car_x;
//            pos_y = car_y;
//            angle = deg2rad(car_yaw);
//          } else {
//            pos_x = previous_path_x[path_size-1];
//            pos_y = previous_path_y[path_size-1];
//
//            double pos_x2 = previous_path_x[path_size-2];
//            double pos_y2 = previous_path_y[path_size-2];
//            angle = atan2(pos_y-pos_y2,pos_x-pos_x2);
//          }

          //to do (follow the car before you):
          //    1. find map waypoints after current position
          //    2. find current position s and d
          //    3. propose path that keeps current d but advances s
          //    4. if path intersect with any vehicles around?
          //int nextPt = NextWaypoint(pos_x, pos_y, angle, map_waypoints_x, map_waypoints_y);
//          double pos_s;
//          double pos_d;
//          vector<double> sd = getFrenet(pos_x, pos_y, angle, map_waypoints_x, map_waypoints_y);
//          pos_s = sd[0];
//          pos_d = sd[1];
//
//          std::cout<<"current pos_x, pos_y: "<<pos_x<<" "<<pos_y<<std::endl;
//          std::cout<<"current pos_s: "<<pos_s<<std::endl;

          const double MAX_V = 49.5/2.24; //my speed limit on mph
          const double DELTA_T = 0.02; //0.02 second per path point
          const double MAX_ACC = 10; //m/s^2
          const double MAX_JERK = 10; //m/s^3

          int curr_lane; //lane = 0, 1, 2
          curr_lane = car_d/4;  //determine current lane based on vehicle d
          std::cout<<"current lane: "<<curr_lane<<std::endl;

          state curr_state = keep_lane;
          //double s_inc= ref_vel*1600.0/3600.0*deta_T; // 50mph -> 22.352 m/s -> 0.447 m/0.02s

          int nPts = 50-path_size;  // append n points from (pos_s, pos_d) to (target_s, target_d)

          // Decision making based on sensor data
          //   rule A: if no object in front (same lane), cruise at ref_vel or ramp up speed to ref_vel
          //           if obj in front and dist / (my_vel-obj_vel) < 3 sec,
          //                if no object on left:
          //                      change lane to left
          //                elif no object on right:
          //                      change lane to right
          //                else:
          //                      slow down to ensure time to collision 3 sec

          // for stay in lane
          // speed target is MAX_V
          // update speed target, if any car in front with time_to_collision < 1 second ((obj_s - curr_s)/speed diff)
          // update speed target if MAX_ACC and MAX_JERK is violated
          double target_v;  // target speed at the end of path planning (1 second later)
          bool left_traffic, right_traffic; // flag if there is traffic to prevent turn left, or right
          left_traffic = false;
          right_traffic = false;
          target_v = std::min(MAX_V, car_speed+MAX_ACC);  //initial target speed
          for(int i= 0; i< sensor_fusion.size(); i++)
          {
            //std::cout<<sensor_fusion[i]<<", ";
        	  auto obj = sensor_fusion[i];
//        	  if(obj["s"]<target_s && obj["s"]>=pos_s && fabs(obj["d"]-target_d)<2)
//        		  target_s = obj["s"];
        	  double obj_vx = obj[3];
        	  double obj_vy = obj[4];
        	  double obj_v = sqrt(obj_vx*obj_vx+obj_vy*obj_vy);
        	  double obj_s = obj[5];
        	  double obj_d = obj[6];
        	  //std::cout<<" obj_s = "<<obj_s<<std::endl;
        	  double ttc = 1.0; //time to collision threshold, in second
        	  if(obj_v<car_speed && fabs(obj_d-car_d)<2 && obj_s>car_s && (obj_s-car_s)/(car_speed-obj_v)<ttc)
        	  {
        	  	  target_v = obj_v;
        	  	  //std::cout<<"!! Update target_s to "<<target_s<<std::endl;
        	  }
        	  if(curr_lane>0 && fabs(obj_d-(car_d-4))<2 &&
        			  ( (obj_s>=car_s && obj_v<car_speed && (obj_s-car_s)/(car_speed-obj_v)<ttc)||
        					  (obj_s<car_s && obj_v>car_speed && (car_s-obj_s)/(obj_v-car_speed)<ttc) ))
        		  left_traffic = true;
        	  if(curr_lane<2 && fabs(obj_d-(car_d+4))<2 &&
        	          			  ( (obj_s>=car_s && obj_v<car_speed && (obj_s-car_s)/(car_speed-obj_v)<ttc)||
        	          					  (obj_s<car_s && obj_v>car_speed && (car_s-obj_s)/(obj_v-car_speed)<ttc) ))
        	      right_traffic = true;

          }
          std::cout<<"target_V = "<<target_v<<std::endl;
          std::cout<<"left_traffic = "<<left_traffic<<std::endl;
          std::cout<<"right_traffic = "<<right_traffic<<std::endl;

          vector<double> target_s(3), target_d(3); // set target way points
          if (target_v <std::min(MAX_V, car_speed+MAX_ACC)) // slow traffic ahead
          {
        	  //if safe to move left and target speed can increase -> move left
        	  if(curr_lane>0 && left_traffic==false)
        	  {
        		  target_s[0] = car_s+30;
				  target_s[1] = car_s+60;
				  target_s[2] = car_s+90;
				  target_d[0] = (2+4*(curr_lane-1));
				  target_d[1] = target_d[0];
				  target_d[2] = target_d[0];
        	  }
        	  else if (curr_lane<2 && right_traffic==false)
        	  {
        		  target_s[0] = car_s+30;
				  target_s[1] = car_s+60;
				  target_s[2] = car_s+90;
				  target_d[0] = (2+4*(curr_lane+1));
				  target_d[1] = target_d[0];
				  target_d[2] = target_d[0];

        	  }
        	  else
        	  {
        		  target_s[0] = car_s+30;
				  target_s[1] = car_s+60;
				  target_s[2] = car_s+90;
				  target_d[0] = (2+4*curr_lane);
				  target_d[1] = target_d[0];
				  target_d[2] = target_d[0];
        	  }

        	  //if safe to move right and target speed can increase -> move right

          }
          else // no traffic ahead -> keep lane
          {
        	  target_s[0] = car_s+30;
        	  target_s[1] = car_s+60;
        	  target_s[2] = car_s+90;
        	  target_d[0] = (2+4*curr_lane);
        	  target_d[1] = target_d[0];
        	  target_d[2] = target_d[0];
          }
          std::cout<<"target_s: ";
          for(int i=0;i<3;i++)
          {
        	  std::cout<<target_s[i]<<"; ";
          }
          std::cout<<std::endl;
          std::cout<<"target_d: ";
			for(int i=0;i<3;i++)
			{
			  std::cout<<target_d[i]<<"; ";
			}
			std::cout<<std::endl;


          //generate trajectory based on spline function
          vector<double> ptsx;
          vector<double> ptsy;
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);
          //generate points to make sure path is tangent to current yaw
          if(path_size<2) //if previous size is almost empty, use the car as starting reference
          {
        	  double prev_car_x = car_x-cos(car_yaw);
        	  double prev_car_y = car_y-sin(car_yaw);
        	  ptsx.push_back(prev_car_x);
        	  ptsx.push_back(car_x);

        	  ptsy.push_back(prev_car_y);
        	  ptsy.push_back(car_y);

          }
          else
          {
        	  ref_x = previous_path_x[path_size-1];
        	  ref_y = previous_path_y[path_size-1];
        	  double ref_x_prev = previous_path_x[path_size-2];
        	  double ref_y_prev = previous_path_y[path_size-2];
        	  ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);

        	  ptsx.push_back(ref_x_prev);
        	  ptsx.push_back(ref_x);

        	  ptsy.push_back(ref_y_prev);
        	  ptsy.push_back(ref_y);
          }

          vector<double> next_wp0 = getXY(target_s[0], target_d[0], map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 = getXY(target_s[1], target_d[1], map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getXY(target_s[2], target_d[2], map_waypoints_s, map_waypoints_x, map_waypoints_y);
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
          //check data
          for(double i:ptsx)
          {  std::cout<<i<<"; ";

          }
          std::cout<<std::endl;
          for(double i:ptsy)
			{  std::cout<<i<<"; ";

			}
          std::cout<<std::endl;

          //create spline
          tk::spline s;
          s.set_points(ptsx, ptsy);

          for(int i=0;i<path_size; i++)
          {
        	  next_x_vals.push_back(previous_path_x[i]);
        	  next_y_vals.push_back(previous_path_y[i]);
          }
          // get points along the spline function
          double target_x = 30.0;
          double target_y = s(target_x);
          double target_dist = sqrt(target_x*target_x+target_y*target_y);
          double x_add_on = 0;
          // fill up the rest of path planner
          for(int i=1; i<=nPts; i++)
          {
        	  double N = (target_dist/(.02*target_v));
        	  double x_point = x_add_on+(target_x)/N;
        	  double y_point = s(x_point);
        	  x_add_on = x_point;
        	  double x_ref = x_point;
        	  double y_ref = y_point;
        	  x_point = (x_ref*cos(ref_yaw)-y_ref*sin(ref_yaw));
        	  y_point = (x_ref*sin(ref_yaw)+y_ref*cos(ref_yaw));
        	  next_x_vals.push_back(x_point+ref_x);
        	  next_y_vals.push_back(y_point+ref_y);
          }
          std::cout<<"check final path points: "<<std::endl;
          for(double i:next_x_vals)
        	  std::cout<<i<<"; ";
          std::cout<<std::endl;
          for(double i:next_y_vals)
			  std::cout<<i<<"; ";
			std::cout<<std::endl;
//          s_inc = (target_s-pos_s)/nPts;
//          for (int i = 0; i < nPts; ++i) {
//        	pos_s += s_inc;
//        	vector<double> xy = getXY(pos_s, pos_d, map_waypoints_s,  map_waypoints_x, map_waypoints_y);
//            next_x_vals.push_back(xy[0]);
//            next_y_vals.push_back(xy[1]);
//          }
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
