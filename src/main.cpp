#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>

// for convenience
using json = nlohmann::json;
using namespace std;

//debug
#define USERDEBUG

#ifdef USERDEBUG
#define Debug(x) cout << x
#else
#define Debug(x)
#endif

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

  PID pid_steer, pid_speed;
  // TODO: Initialize the pid variable.

  //debug info
  Debug( "[main]: Initialization begin: ====================" << endl);


  ///* Tuning reference
  /* tune the parameters by trial-and-error with refernce:
   * https://www.wescottdesign.com/articles/pid/pidWithoutAPhd.pdf
   * tuning process can be shown as below
   * Step 0: set Ki = Kd = 0, set a low Kp without oscillation
   * Step 1: TUNE Kd:
   *          Set Kd = 100*Kp as a starting point.
   *          If no oscillation, set Kd = Kd*2; If oscillation, set Kd = Kd/2;
   *          Once close to oscillation, fine tune Kd = Kd/2;
   * Step 2: TUNE Kp:
   *          Set Kp = Kd/100 as a starting point.
   *          If no oscillation, set Kp = Kp*10; If oscillation, set Kp = Kp/10;
   *          Once close to oscillation, fine tune Kp = Kp/2;
   * Step 3: TUNE Ki:
   *          Set Ki = (Kp/Kd) * Kp as a starting point.
   *          If no oscillation, set Ki= Ki*10; If oscillation, set Ki = Ki/10;
   *          Once close to oscillation, fine tune Ki = Ki/2;
   */

  /* My tuning for steer
   * Step 0.0: set Kp = 1, Ki = 0, Kd = 0. results: large oscillation --> unstable
   * Step 0.1: set Kp = 0.1, Ki = 0, Kd = 0. results: small oscillation --> unstable
   * Step 0.2: set Kp = 0.01, Ki = 0, Kd = 0. results: no oscillation ,but the system cannot respond quickly
   * Step 1.0: set Kp = 0.01, Ki = 0, Kd = 1. results: large oscillation --> unstable
   */
  double steer_Kp = 0.01;
  double steer_Ki = 0.0;
  double steer_Kd = 1.0;
  double steer_output = 1.0;
  pid_steer.Init(steer_Kp, steer_Ki, steer_Kd, steer_output);

  //pid_speed.Init(double Kp = 0.1, double Ki = 1.0, double Kd = 0.0001);

  //debug info
  Debug( "[main]: pid for steer are set as following: " << endl);
  Debug( "[main]: Kp_ = " << pid_steer.Kp_ << endl);
  Debug( "[main]: Ki_ = " << pid_steer.Ki_ << endl);
  Debug( "[main]: Kd_ = " << pid_steer.Kd_ << endl);


  h.onMessage([&pid_steer](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
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
          //double speed = std::stod(j[1]["speed"].get<std::string>());
          //double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double steer_value;
          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */
          pid_steer.UpdateError(cte = cte);
          steer_value = pid_steer.TotalError();
          if(steer_value > 1.0){
              steer_value = 1.0;
          }else if(steer_value < -1.0){
              steer_value = -1.0;
          }
          
          // DEBUG
          std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = 0.3;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
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
