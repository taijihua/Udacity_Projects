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
          car_speed = car_speed/2.24; //convert car speed from mph to m/s

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

          const double MAX_V = 49.5/2.24; //speed limit in m/s
          const double DELTA_T = 0.02; //0.02 second per path point
          const double MAX_ACC = 10*0.7; //max acceleration in m/s^2, use 70% to give some buffer room
          const double MAX_JERK = 10*0.8; //max jerk in m/s^3, currently NOT USED

          int curr_lane; //current lane, = 0, 1, 2
          curr_lane = car_d/4;  //determine current lane based on vehicle's d
          static int target_lane = curr_lane; //target lane = 0, 1, 2, it's the lane vehicle will switch to or stay on
          //std::cout<<"current lane: "<<curr_lane<<std::endl;
          static state curr_state = keep_lane;

          //double pos_x;
          //double pos_y;
          //double angle;
          int prev_path_size = previous_path_x.size(); //only keep up to 5 previous history points
          if(prev_path_size>5)
              prev_path_size = 5;

          vector<double> ptsx; //waypoints for trajectory generation
          vector<double> ptsy; //waypoints for trajectory generation
          double ref_x = car_x;
		  double ref_y = car_y;
		  double ref_yaw = deg2rad(car_yaw);
		  double ref_v = car_speed;
		  //generate points to make sure path is tangent to current yaw
		  if(prev_path_size<2) //if previous size is almost empty, use the car as starting reference
		  {
		      double prev_car_x = car_x-cos(car_yaw);
			  double prev_car_y = car_y-sin(car_yaw);
			  ptsx.push_back(prev_car_x);
			  ptsx.push_back(car_x);

			  ptsy.push_back(prev_car_y);
			  ptsy.push_back(car_y);
		  }
		  else // use previous path's end point as reference
		  {
			  ref_x = previous_path_x[prev_path_size-1];
			  ref_y = previous_path_y[prev_path_size-1];
			  double ref_x_prev = previous_path_x[prev_path_size-2];
			  double ref_y_prev = previous_path_y[prev_path_size-2];
			  ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);
			  ref_v = sqrt(pow(ref_y-ref_y_prev, 2)+pow(ref_x-ref_x_prev, 2))/DELTA_T;

			  ptsx.push_back(ref_x_prev);
			  ptsx.push_back(ref_x);

	  		  ptsy.push_back(ref_y_prev);
			  ptsy.push_back(ref_y);
		  }

          int nPts = 50-prev_path_size;  // append nPts points to make up total 50 points

          //  based on sensor data, determine traffic on left/right, and target velocity on each lane
          double target_v[3];  // target speed at the end of path planning (1 second later)
          bool left_traffic, right_traffic; // flag if there is traffic to prevent move left, or right
          left_traffic = false;
          right_traffic = false;
          target_v[0] = std::min(MAX_V, ref_v+MAX_ACC);  //initial value for target speed
          target_v[1] = std::min(MAX_V, ref_v+MAX_ACC);  //initial value for target speed
          target_v[2] = std::min(MAX_V, ref_v+MAX_ACC);  //initial value for target speed
          for(int i= 0; i< sensor_fusion.size(); i++)
          {
        	  auto obj = sensor_fusion[i];
        	  double obj_vx = obj[3];
        	  double obj_vy = obj[4];
        	  double obj_v = sqrt(obj_vx*obj_vx+obj_vy*obj_vy);
        	  double obj_s = obj[5];
        	  double obj_d = obj[6];
        	  //std::cout<<" obj_s = "<<obj_s<<std::endl;
        	  double ttc = 2.7; //time to collision threshold, in second, manual tweek this to determine car following distance
        	  double safe_dist = 3; //safe distance to keep away from front vehicle

        	  // if current lane has slow traffic ahead, update target speed
        	  if(obj_v<target_v[curr_lane] && fabs(obj_d-(curr_lane*4+2))<2 && obj_s>car_s &&
        			 ((obj_s-car_s)/(target_v[curr_lane]-obj_v)<ttc ||(obj_s-car_s)<safe_dist))
        	  {
        	  	  target_v[curr_lane] = obj_v;
        	  	  //std::cout<<"!! Update target_s to "<<target_s<<std::endl;
        	  }
        	  // if left lane has slow traffic ahead, update target speed, and also update left traffic flag if object closeby
        	  if(curr_lane>0)
        	  {
        		  if (fabs(obj_d-((curr_lane-1)*4+2))<2) //left lane has traffic
        		  {
        			  double speed = (ref_v+target_v[curr_lane-1])/2; // use average speed between now and 1 second later for judging collision risk
        			  // slow traffic ahead
        			  if(obj_s>=car_s && obj_v<speed &&
        					  (((obj_s-car_s)/(speed-obj_v)<ttc)||(obj_s-car_s)<safe_dist))
        				  target_v[curr_lane-1] = obj_v; // set lower speed target for left lane
        			  //if((obj_s<car_s && obj_v>speed && (car_s-obj_s)/(obj_v-speed)<ttc))
        			  if(fabs(obj_s-car_s)<8)
        				  left_traffic = true;
        		  }
        	  }
        	  // if right lane has slow traffic ahead
        	  if(curr_lane<2)
        	  {
        		  if (fabs(obj_d-((curr_lane+1)*4+2))<2) //left lane has traffic
				  {
					  double speed = (ref_v+target_v[curr_lane+1])/2; // use average speed between now and 1 second later for judging collision risk
					  // slow traffic ahead
					  if(obj_s>=car_s && obj_v<speed &&
							  (((obj_s-car_s)/(speed-obj_v)<ttc)||(obj_s-car_s)<safe_dist))
						  target_v[curr_lane+1] = obj_v; // set lower seep target for right lane
					  //if((obj_s<car_s && obj_v>speed && (car_s-obj_s)/(obj_v-speed)<ttc)) // disable lane change
					  if(fabs(obj_s-car_s)<8)
						  right_traffic = true;
				  }
        	  }

          }
          /*
          std::cout<<"curr_lane = "<<curr_lane<<std::endl;
          std::cout<<"target speeds: ";
          for(int i=0;i<3;i++)
        	  std::cout<<target_v[i]<<", ";
          std::cout<<std::endl;
          std::cout<<"left_traffic = "<<left_traffic<<std::endl;
          std::cout<<"right_traffic = "<<right_traffic<<std::endl;
          */

          //state transitions
          if (curr_state == keep_lane)
          {
			  if (target_v[curr_lane] <std::min(MAX_V, ref_v+MAX_ACC)) // slow traffic ahead
			  {
				  //if safe to move left and target speed can increase -> move left
				  if(curr_lane>0 && left_traffic==false)
				  {
					  if(target_v[curr_lane-1]>target_v[curr_lane])
					  {

						  target_lane = curr_lane-1;
						  curr_state = move_left;
					  }
				  }
				  else if (curr_lane<2 && right_traffic==false)
				  {
					  if(target_v[curr_lane+1]>target_v[curr_lane])
					  {
						  target_lane = curr_lane+1;
						  curr_state = move_right;
					  }

				  }
				  else
				  {
					  target_lane = curr_lane;
					  curr_state = keep_lane;
				  }

			  }
          }
          else if (curr_state==move_left)
          {
        	  // successfully changed lane (crossed lane line)
        	  if (target_lane == curr_lane)
        		  curr_state = keep_lane;
          }
          else if(curr_state== move_right)
          {
        	  // successfully changed lane (crossed lane line)
        	  if (target_lane == curr_lane)
        		  curr_state = keep_lane;

          }

          vector<double> target_s(3), target_d(3); // set target way points
		  target_s[0] = car_s+30;
		  target_s[1] = car_s+60;
		  target_s[2] = car_s+90;
		  target_d[0] = (2+4*target_lane);
		  target_d[1] = target_d[0];
		  target_d[2] = target_d[0];

          // continue adding waypoints for path planning
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

          //create spline
          tk::spline s;
          s.set_points(ptsx, ptsy);

          for(int i=0;i<prev_path_size; i++)
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
          double v0;
		  v0 = ref_v;
          double acc;
		  acc = (target_v[target_lane]-v0)*double(nPts)/50; // default 1 second interval
		  //std::cout<<"current state: "<<curr_state<<std::endl;
		  //std::cout<<"target_lane: "<<target_lane<<", target_v[target_lane] = "<<target_v[target_lane]<<std::endl;
          acc = std::min(acc, MAX_ACC);
          acc = std::max(acc, -MAX_ACC);
          //std::cout<<"car_speed = "<<car_speed<<std::endl;
          //std::cout<<"acceleration = "<<acc<<std::endl;
          for(int i=1; i<=nPts; i++)
          {

        	  //linearly ramp up the speed to target_v
        	  //v0 = ref_v+(target_v[target_lane]-ref_v)*i/nPts;
        	  //std::cout<<"v0: "<<v0<<std::endl;
        	  //double N = (target_dist/(DELTA_T*v0));
        	  //double x_point = x_add_on+(target_x)/N;

        	  double x_point = x_add_on+v0*DELTA_T+0.5*acc*DELTA_T*DELTA_T;
        	  v0 = v0+acc*DELTA_T;

        	  x_add_on = x_point;

        	  double y_point = s(x_point);

        	  double temp_x = x_point;
       	  	  double temp_y = y_point;
        	  x_point = (temp_x*cos(ref_yaw)-temp_y*sin(ref_yaw));
        	  y_point = (temp_x*sin(ref_yaw)+temp_y*cos(ref_yaw));

        	  next_x_vals.push_back(x_point+ref_x);
        	  next_y_vals.push_back(y_point+ref_y);
          }
          //std::cout<<"==========end one cycle========"<<std::endl;
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
