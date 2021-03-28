#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include "json.hpp"
#include "PID.h"
#include "twiddle_machine.h"

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


    //Best after twidle 3 - action*1.0 //Trying to make it smoother at turns
    double steeringInitP = 0.643459;
    double steeringInitI = 0.0;
    double steeringInitD = 2.0941;
    double dt = 1.0;
    double maxValue = 0.45;
    double maxISum = 0.05;
    double alpha = 0.2;
    
    //Steering controller
    PID steeringPid;
    steeringPid.Init( steeringInitP, steeringInitI, steeringInitD, dt, maxValue, maxISum, alpha );

    std::vector< double > steeringParams{ steeringInitP,      steeringInitD};
    std::vector< double > steeringDp    { steeringInitP*0.5,  steeringInitD*0.5 };
    
    //Twiddle - 3700 values. Aprox 1 lap on the track
    TwiddleStateMachine tdSteering( steeringParams.size(), 3700, steeringParams, steeringDp );


    //Throttle Controller
    PID throttlePid;
    throttlePid.Init( 0.06, 0.0, 0.0, dt, 1.0, 1.0, 1.0 ); 

    double steer_value;
    double throttle_value;

    h.onMessage([&steeringPid, &throttlePid, &tdSteering, &steer_value, &throttle_value](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, 
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
                    double angle = std::stod(j[1]["steering_angle"].get<string>());
              

                    steeringPid.UpdateError( cte ); 
                    const double new_steering_value = steeringPid.GetOutput();
                    const double steering_delta = new_steering_value - steer_value;
                    steer_value = new_steering_value;

              
                    //Updating PID Values for next iterations
                    tdSteering.update( cte, steering_delta*4.0 );
                    const double &kp = tdSteering.params()[0];
                    //const double &ki = tdSteering.params()[1];
                    const double &kd = tdSteering.params()[1];
                    steeringPid.UpdateConstants( kp, 0.0, kd );


                    const double speedError = ( speed - 15.0 );
                    throttlePid.UpdateError( speedError );
                    throttle_value = throttlePid.GetOutput();

                    json msgJson;
                    msgJson["steering_angle"] = steer_value;
                    msgJson["throttle"] = throttle_value;
                    auto msg = "42[\"steer\"," + msgJson.dump() + "]";
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