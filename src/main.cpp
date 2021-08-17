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
    } else if (b1 != string::npos && b2 != string::npos) {
        return s.substr(b1, b2 - b1 + 1);
    }
    return "";
}

int main() {
    uWS::Hub h;

    PID pid;
    PID pidSpeed;
    /**
     * TODO: Initialize the pid variable.
     */
    // PID: Kp: 0.5 Kd: 6.1 Ki: 5.1e-08 #1
    // PID Speed : Kp: 3 Kd: 3.5 Ki: 5.1e-08 #1
    pid.Init(0.5, 0.000000051, 6.1, 0.02, 0.01, 0.00000001, 0.2);
    pidSpeed.Init(3, 0.0000000051, 3.5, 1, 1, 0.000000001, 0.001);

    static double const ROAD_OFFSET = 0.76;
    static double const MAX_SPEED = 0.3;
    static double const MIN_SPEED = 0.1;

    int loop_counter = 1;
    double mean_err_cte = 0;
    double mean_err_speed = 0;
    double mean_error = 0;

    static bool TRAIN_PARAMETER_SPEED_PID = false;
    static bool TRAIN_PARAMETER_STEERING_PID = false;

    h.onMessage([&pid, &pidSpeed, &loop_counter, &mean_err_speed, &mean_error, &mean_err_cte](
            uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
            uWS::OpCode opCode) {
        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
        if (length > 2 && data[0] == '4' && data[1] == '2') {
            auto s = hasData(string(data).substr(0, length));

            if (!s.empty()) {
                auto j = json::parse(s);

                string event = j[0].get<string>();

                if (event == "telemetry") {
                    // j[1] is the data JSON object
                    double cte = std::stod(j[1]["cte"].get<string>());
                    double speed = std::stod(j[1]["speed"].get<string>());
//                    double angle = std::stod(j[1]["steering_angle"].get<string>());
                    double steer_value;
                    /**
                     * TODO: Calculate steering value here, remember the steering value is
                     *   [-1, 1].
                     * NOTE: Feel free to play around with the throttle and speed.
                     *   Maybe use another PID controller to control the speed!
                     */

                    // Add road offset to stay in middle
                    cte -= ROAD_OFFSET;

                    pid.UpdateError(cte);
                    pidSpeed.UpdateError(cte);


                    /**
                     * Train PID controller (only one at a time)
                     * Retrain triggered when reconnect to the simulator
                     */
                    if (TRAIN_PARAMETER_SPEED_PID) {
                        if (loop_counter == -1) {
                            bool result = pidSpeed.TrainParameters(mean_err_cte);
                            if (result) {
                                std::cout << "========= Parameter training complete =========" << std::endl;
                            }

                            // Reset parameters
                            mean_error = 0;
                            mean_err_cte = 0;
                            mean_err_speed = 0;
                        }
                    } else if (TRAIN_PARAMETER_STEERING_PID) {
                        if (loop_counter == -1) {
                            bool result = pid.TrainParameters(mean_err_cte);
                            if (result) {
                                std::cout << "========= Parameter training complete =========" << std::endl;
                            }
                            // Reset parameters
                            mean_error = 0;
                            mean_err_cte = 0;
                            mean_err_speed = 0;
                        }
                    }

                    if (loop_counter >= 0) {
                        // Add error to the power of 2 (negative values in respective)
                        mean_err_cte += pow(cte, 2);
                    }
                    // Increment loop counter
                    loop_counter = (loop_counter + 1);


                    steer_value = pid.TotalError();
                    double speed_value = 1 - pidSpeed.TotalError();

                    mean_err_speed += speed_value;

                    if (steer_value > 1) {
                        steer_value = 1;
                    } else if (steer_value < -1) {
                        steer_value = -1;
                    }

                    if (speed_value > MAX_SPEED) {
                        speed_value = MAX_SPEED;
                    } else if (speed_value < MIN_SPEED) {
                        speed_value = MIN_SPEED;
                    }

                    // DEBUG
//                    std::cout << "CTE: " << cte << " Steering Value: " << steer_value << " Speed: " << speed_value
//                              << std::endl;

                    json msgJson;
                    msgJson["steering_angle"] = steer_value;
                    msgJson["throttle"] = speed_value;
                    auto msg = "42[\"steer\"," + msgJson.dump() + "]";
//                    std::cout << msg << std::endl;
                    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                }  // end "telemetry" if
            } else {
                // Manual driving
                string msg = "42[\"manual\",{}]";
                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            }
        }  // end websocket message if
    }); // end h.onMessage

    h.onConnection(
            [&h, &loop_counter, &mean_error, &mean_err_cte, &mean_err_speed, &pidSpeed, &pid](
                    uWS::WebSocket<uWS::SERVER> ws,
                    uWS::HttpRequest req) {
                std::cout << "Connected!!!" << std::endl;
                pidSpeed.PrintParameters();
                pid.PrintParameters();

                /**
                 * Start training loop
                 */
                mean_err_speed /= loop_counter;
                mean_err_cte /= pow(loop_counter, 2);

                mean_error = 0.5 * mean_err_cte + 0.5 * mean_err_speed;
                std::cout << "[Speed PID] Car moved for " << loop_counter << " with a mean error of " << mean_err_cte
                          << std::endl;

                loop_counter = -1;
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