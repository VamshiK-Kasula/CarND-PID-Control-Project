#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include "json.hpp"
#include "PID.h"
#include <limits>

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
int main()
{
  uWS::Hub h;

  PID pid;
  int iteration = 0;
  int total_iterations = 600;
  bool twiddle = false;
  double p[3] = {0.198004, 0.00095, 1.2535};
  double dp[3] = {.01, .0001, .1};
  double accumulated_cte = 0.0;
  double error = 0.0;
  double best_error = std::numeric_limits<double>::infinity();
  double tolerance = 0.1;
  int idx = 0;
  int total_iterator = 0;
  int sub_move = 0;
  bool twiddle_initialization = true;
  bool twiddle_update = true;
  double updated_p[3] = {p[0],p[1],p[2]};


  pid.Init(p[0],p[1],p[2]);
  h.onMessage([&pid, &p, &dp, &iteration, &total_iterations, &tolerance, &error, &best_error, &idx, &total_iterator, &accumulated_cte, &twiddle_initialization, &sub_move, &twiddle_update, &twiddle, &updated_p](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double steer_value;
          json msgJson;
          
          if (twiddle == true)
          {
            if(iteration==0)
            {              
              pid.Init(p[0],p[1],p[2]); // by trail and error
            }
            pid.UpdateError(cte);
            steer_value = pid.TotalError();
            accumulated_cte += pow(cte,2);
            
            iteration = iteration+1;
            if (iteration > total_iterations)
            { 
              if(twiddle_initialization == true) 
              {
                p[idx%3] += dp[idx%3];
                twiddle_initialization = false;
              }
              else
              {
                error = accumulated_cte/total_iterations;
                
                if(error < best_error && twiddle_update == true) 
                {
                    best_error = error;
                    updated_p[0] = p[0];
                    updated_p[1] = p[1];
                    updated_p[2] = p[2];
                    dp[idx%3] *= 1.1;
                    sub_move += 1;
                }
                else
                {
                  if(twiddle_update == true) 
                  {
                    p[idx%3] -= 2 * dp[idx%3];
                    twiddle_update = false;
                  }
                  else 
                  {
                    if(error < best_error)
                     {
                        best_error = error;
                        updated_p[0] = p[0];
                        updated_p[1] = p[1];
                        updated_p[2] = p[2];
                        dp[idx%3] *= 1.1;
                        sub_move += 1;
                    }
                    else 
                    {
                        p[idx%3] += dp[idx%3];
                        dp[idx%3] *= 0.9;
                        sub_move += 1;
                    }
                  }
                }
                
              }              

              if(sub_move > 0) 
              {
                idx = idx+1;
                twiddle_initialization = true;
                twiddle_update = true;
                sub_move = 0;
              }

              accumulated_cte = 0.0;
              iteration = 0;
              total_iterator = total_iterator+1;

              if( dp[0]+dp[1]+dp[2] > tolerance) 
              {
                std::cout <<"Kp :" <<updated_p[0] <<"\tKi :" <<updated_p[1] <<"\tKd :" <<updated_p[2] <<std::endl;
                std::string reset_msg = "42[\"reset\",{}]";
                ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
              }
              
            } 
            else 
            {
              msgJson["steering_angle"] = steer_value;
              msgJson["throttle"] = 0.3;
              auto msg = "42[\"steer\"," + msgJson.dump() + "]";
              ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            }
           
          } else { //twiddle if
            pid.UpdateError(cte);
            steer_value = pid.TotalError();
        
            // DEBUG
            std::cout << "CTE: " << cte << " Steering Value: " << steer_value << " Count: " << iteration << std::endl;
            msgJson["steering_angle"] = steer_value;
            msgJson["throttle"] = 0.3;
            auto msg = "42[\"steer\"," + msgJson.dump() + "]";
            std::cout << msg << std::endl;
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          } //twiddle else
        }//telemtery
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
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
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  
  h.run();
}