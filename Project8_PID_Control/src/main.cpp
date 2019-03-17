#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include "json.hpp"
#include "PID.h"

// for convenience
using nlohmann::json;
using std::string;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != string::npos) {
    return "";
  }
  else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main() {
  uWS::Hub h;

  PID pid;  // PID controller for steering
  static PID pid_speed; //PID controller for speed, use static declaration to pass into the onMessage() function
  static uint ct = 0;  //count of iterations, used for updating parameters in twiddle mode
  //pid.Init(2.0408, 0, 7.72598);
  //pid.Init(0.1, -0.9, 1);
  //pid.Init(0.821031, 0, 2.44292);
  //pid.Init(0.787197, 0, 2.73721);
  //pid.Init(0.239278, 0, 2.94026);
  //pid.Init(0.239278, 0, 2.61221);
  pid.Init(0.239278, 0, 3.36108); //from twiddle algo with best err
  //pid_speed.Init(0, 0, 0, 50);
  pid_speed.Init(5.75527, 0.00735012, -0.0515436, 25); //from twiddle algo with best err

  //implement twiddle
  static bool twiddleMode = false;  //change this flag to turn on/off twiddle mode
  ///====================The following section defines parameters only used in twiddle mode==========
  static PID& pid_twiddle = pid;  //reference to the pid controller for twiddle algorithm, value can be either pid or pid_speed
  //static PID& pid_twiddle = pid_speed;  //reference to the pid controller for twiddle algorithm, value can be either pid or pid_speed
  static const double targetVal = 0.0;  // target value, if steering use 0, if speed use 50

  static double p[3] = {0, 0, 0};  //initial PID parameters to kickoff twiddle
  static double dp[3] = {1, 1, 1}; //initial search step for PID parameters
  //static double p[3] = {0.239278, 0, 2.61221};
  //static double dp[3] = {1, 1, 1}; //initial search step for PID parameters
  static double bestP[3] = {0, 0, 0}; // to store the best PID parameters

  static double best_err = std::numeric_limits<double>::infinity(); //first set best_err to infinity
  static int ix = 0; // index of parameter to be updated in twiddle, value = 0, 1, 2
  static bool tryAgain = true; //flag indicating a second trial in twiddle mode

  if(twiddleMode)
  {
	  pid_twiddle.Init(p[0], p[1], p[2], targetVal);
  }
  //=================== End of twiddle parameter definitions=======

  h.onMessage([&pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(string(data).substr(0, length));

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object

          double cte = std::stod(j[1]["cte"].get<string>());
          double speed = std::stod(j[1]["speed"].get<string>());
          //double angle = std::stod(j[1]["steering_angle"].get<string>());
          double steer_value;
          /**
           * TODO: Calculate steering value here, remember the steering value is
           *   [-1, 1].
           * NOTE: Feel free to play around with the throttle and speed.
           *   Maybe use another PID controller to control the speed!
           */
          ct += 1; //increment iteration count
          pid.UpdateError(cte);
          pid.TotalError();
          //std::cout<<"cte: "<<cte<<", ";
          steer_value = pid.CalcOutput();
          //steer_value = deg2rad(steer_value);

          double throttle_value=0.3;
          pid_speed.UpdateError(speed);
          pid_speed.TotalError();
          throttle_value = pid_speed.CalcOutput();
          
          // DEBUG
//          std::cout<<"======="<<std::endl;
//          std::cout << "CTE: " << cte << " Steering Value: " << steer_value
//                    << std::endl;
//          std::cout << "total error: " << pid.TotalError() <<std::endl;
//          std::cout << "Speed: " << speed << " throttle: " << throttle_value
//                              << std::endl;
//          std::cout << "total error(speed): " << pid_speed.TotalError() <<std::endl;


          json msgJson;
          msgJson["steering_angle"] = steer_value;

          //msgJson["throttle"] = 0.3;  //0.3
          msgJson["throttle"] = throttle_value;

          auto msg = "42[\"steer\"," + msgJson.dump() + "]";

          //reset between twiddle iterations
          if(ct%200==0 && twiddleMode)  // if twiddle mode, update parameters every 200 iterations and reset simulation to start
          {
        	  int it = ct/200;
        	  double err = pid_twiddle.TotalError();
        	  std::cout<<"===========iteration: "<<it<<std::endl;
        	  std::cout<<"Last PID coefficients: ["<<p[0]<<", "<<p[1]<<", "<<p[2]<<"]"<<std::endl;
        	  std::cout<<"dp: ["<<dp[0]<<", "<<dp[1]<<", "<<dp[2]<<"]"<<std::endl;
        	  if(it==1) //first round is [0, 0, 0]
        	  {
        		  best_err = err;
        		  for(int i=0;i<3;i++)
        			  bestP[i] = p[i];
        		  p[ix] += dp[ix];
        		  pid_twiddle.Init(p[0], p[1], p[2], targetVal);
        	  }
        	  else //other rounds go here
			  {
				  if (err<best_err)
				  {
					  best_err = err;
					  for(int i=0;i<3;i++)
					      bestP[i] = p[i];
					  dp[ix] *= 1.1;
					  ix = (ix+1)%3;

					  p[ix] += dp[ix];
					  pid_twiddle.Init(p[0], p[1], p[2], targetVal);
					  tryAgain = true;
				  }
				  else if (tryAgain)
				  {
					  p[ix] -= 2 * dp[ix];
					  pid_twiddle.Init(p[0], p[1], p[2], targetVal);
					  tryAgain = false;
				  }
				  else
				  {
					  p[ix] += dp[ix];
					  dp[ix] *= 0.9;
					  ix = (ix+1)%3;

					  p[ix] += dp[ix];
					  pid_twiddle.Init(p[0], p[1], p[2], targetVal);
					  tryAgain = true;
				  }
			  }
        	  std::cout<<"currentError: "<<err<<" bestError: "<<best_err<<std::endl;
        	  std::cout<<"best PID coefficients: ["<<bestP[0]<<", "<<bestP[1]<<", "<<bestP[2]<<"]"<<std::endl;

        	  if(dp[0]+dp[1]+dp[2]<0.001)
        		  return;
        	  msg = "42[\"reset\",{}]";
          }
          //std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket message if
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
