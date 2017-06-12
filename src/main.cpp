#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>

// DEBUG
#define DEBUG

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main()
{
  uWS::Hub h;

  // PID for steering angle
  PID pid;
  // Initialize the pid variable.
  const double Kp = .25;
  const double Kd = 10;
  const double Ki = .0001;
  
  pid.Init(Kp, Ki, Kd);
  
  // PID for throttle
  PID pid2;
  // Initialize the pid variable.
  const double Kp2 = .2;
  const double Kd2 = 9;
  const double Ki2 = .0005;
  
  pid2.Init(Kp2, Ki2, Kd2);
  
  // A flag for differential CTE initialization
  bool is_initialized = false;

  h.onMessage([&pid, &pid2, &is_initialized](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
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
          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */
          
          // Speed PID
          const double REF_SPEED = 30; // Reference speed
          double speed_err = speed - REF_SPEED; // Speed error
          double throttle_value; // Control force for speed
          
          if (!is_initialized)
          {
            pid.d_error_ = cte;
            pid2.d_error_ = speed_err;
            is_initialized = true;
            std::cout << "d_error_ init!" << std::endl;
          }
          
          pid.UpdateError(cte);
          steer_value = pid.TotalError();

          pid2.UpdateError(speed_err);
          // throttle_value = 0.3;
          throttle_value = pid2.TotalError();
          
          // DEBUG
          #ifdef DEBUG
          std::cout << "CTE: " << cte << " | " 
                    << "Steering Value: " << steer_value << " | " 
                    << "Measured angle: " << angle << " | " 
                    << "Measured speed: " << speed << " | " 
                    << std::endl
                    << "pid.p_error_: " << pid.p_error_ << " | " 
                    << "pid.d_error_: " << pid.d_error_ << " | " 
                    << "pid.i_error_: " << pid.i_error_ << " | " 
                    << std::endl;
          #endif
          
          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          #ifdef DEBUG
          std::cout << msg << std::endl;
          #endif
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
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

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
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
