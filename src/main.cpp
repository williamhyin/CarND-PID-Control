#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include "json.hpp"
#include "PID.h"

// for convenience
using nlohmann::json;
using std::string;
using namespace std;

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
    /**
     * TODO: Initialize the pid variable.
     */

    vector<double> p = {0.15000, 0.00100,1.70000}; // initial p vector -> obtaining rough parameter combinations by manually tuning
    vector<double> dp = {0.01000, 0.00010, 0.1000};
    vector<double> p_best = {p[0],p[1],p[2]}; // initial best p vector

    bool is_twiddle = false; // whether to use the twiddle algorithm
    bool first_drive = true; // run first vehicle loop to do first trying:  p[p_index] + dp[p_index]
    bool second_drive = true; // after run first loop, calculate the corresponding error, and find whether the first_drive turning is suitable, if not negative turning
    bool move = false; // whether to move to next parameter of vector P

    int n = 0; // start/reset point of simulator, each cycle of simulator -> n+1
    int n_begin = 0; // where to start to calculate error
    int n_end = 700; // where to end to calculate error and reset the simulator
    int p_ind = 0; // index of parameter of vector p
    int iteration = 0; // how many vehicle loops of simulator have runed

    double total_error = 0.0;
    double error = 0.0;
    double best_error = 999999;
    double tolerance =0.1; // whether sum of dp < tolerance

    bool best_parameter = false; // whether best parameters of p are found


    if(is_twiddle) {
        pid.Init(p[0],p[1],p[2]);

    }else {
        pid.Init(0.06, 0.00031, 1.29);

    }

    h.onMessage(
            [&pid, &n, &n_begin, &n_end, &total_error, &error, &first_drive, &second_drive, &is_twiddle, &p, &p_ind, &dp, &best_parameter, &p_best, &best_error, &iteration, &move, &tolerance](
                    uWS::WebSocket <uWS::SERVER> ws, char *data, size_t length,
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
                            double steer_value;

                            json msgJson;

                            /**
                             * TODO: Calculate steering value here, remember the steering value is
                             *   [-1, 1].
                             * NOTE: Feel free to play around with the throttle and speed.
                             *   Maybe use another PID controller to control the speed!
                             */

                            if (is_twiddle) {
                                int n_interval = n_end - n_begin;
                                // initialize the PID when restart the simulator
                                if (n == 0) {
                                    pid.Init(p[0], p[1], p[2]);
                                }
                                // calculate the sum of error
                                if (n > n_begin && n < n_end) {
                                    total_error += cte * cte;
                                }
                                // update the PID and calculate steer value which should been [-1,1]
                                pid.UpdateError(cte);
                                steer_value = pid.TotalError();

                                if (steer_value > 1) {
                                    steer_value = 1;
                                } else if (steer_value < -1) {
                                    steer_value = -1;
                                }
                                // go next point/next cycle
                                n += 1;
                                // after finished each simulator loop, then use twiddle
                                if (n > n_end) {
                                    if (iteration==0) {
                                        best_error = total_error / n_interval;
                                    }
                                    // whether p should be increased
                                    if (first_drive) {
                                        p[p_ind] += dp[p_ind];
                                        first_drive = false;
                                    } else {
                                        error = total_error / n_interval;
                                        // whether the increased p reduces the error and whether to go into second drive
                                        if (error < best_error && second_drive) {// second_drive is initialized as true
                                            p_best[0] = p[0];
                                            p_best[1] = p[1];
                                            p_best[2] = p[2];
                                            best_error = error;
                                            dp[p_ind] *= 1.1;
                                            move = true;
                                        } else {
                                            if (second_drive) {// before second_drive, reduce p
                                                p[p_ind] -= 2 * dp[p_ind];
                                                second_drive = false;
                                            } else {// after second_drive, check the error again
                                                if (error < best_error) {
                                                    p_best[0] = p[0];
                                                    p_best[1] = p[1];
                                                    p_best[2] = p[2];
                                                    best_error = error;
                                                    dp[p_ind] *= 1.1; // dp increase
                                                    move = true;

                                                } else {
                                                    p[p_ind] += dp[p_ind];
                                                    dp[p_ind] *= 0.9; //dp reduce
                                                    move = true;
                                                }
                                            }
                                        }
                                    }
                                    //  if the corresponding check have been complete,  move to next index,
                                    if (move) {
                                        p_ind += 1;
                                        cout << "move_index: " << p_ind << endl;
                                        first_drive = true;
                                        second_drive = true;
                                        move = false;
                                    } else {
                                        cout << "stay_index " << p_ind << endl;
                                    }
                                    // p[0], p[1], p[2]
                                    if (p_ind == 3) {
                                        p_ind = 0;
                                    }

                                    double sum_dp = dp[0] + dp[1] + dp[2];

                                    if (sum_dp < tolerance) {
                                        best_parameter = true;
                                    }
                                    n = 0;
                                    iteration += 1;
                                    total_error = 0.0;

                                    if (n == 0 && !best_parameter) {

                                        std::string reset_msg = "42[\"reset\",{}]";
                                        ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
                                    } else if (best_parameter) {
                                        std::cout << "find best p" << std::endl;
                                        std::cout << "best p" << p_best[0] << "," << p_best[1] << "," << p_best[2]<< ", "<< std::endl;
                                    }

                                    std::cout << "iteration: " << iteration << std::endl;
                                    std::cout << "best_error: " << best_error << std::endl;
                                    std::cout << "current best p up till now: " << p_best[0] << ", " << p_best[1]<< ", "<< p_best[2] << ", " << std::endl;
                                    std::cout << "sum of dp: " << sum_dp << std::endl;
                                    if (first_drive) {
                                        std::cout << "first drive: " << " next p index: "<<p_ind << " current p value: " << p[0] << ", "<< p[1]<< ", "<< p[2]<< std::endl;
                                    } else {
                                        std::cout << "second drive: " << " next p index: "<<p_ind << " current p value: " << p[0] << ", "<< p[1]<< ", "<< p[2]<< std::endl;
                                    }
                                    cout << endl;

                                } else {
//                            std::cout << "CTE: " << cte << " Steering Value: " << steer_value
//                                      << std::endl;
                                    msgJson["steering_angle"] = steer_value;
                                    msgJson["throttle"] = 0.3;
                                    auto msg = "42[\"steer\"," + msgJson.dump() + "]";
                                    // std::cout << msg << std::endl;
                                    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                                }

                            } else {

                                pid.UpdateError(cte);
                                steer_value = pid.TotalError();
                                if (steer_value > 1) {
                                    steer_value = 1;
                                } else if (steer_value < -1) {
                                    steer_value = -1;
                                }


                                // DEBUG
                                std::cout << "CTE: " << cte << " Steering Value: " << steer_value
                                          << std::endl;

                                msgJson["steering_angle"] = steer_value;
                                msgJson["throttle"] = 0.3;
                                auto msg = "42[\"steer\"," + msgJson.dump() + "]";
                                std::cout << msg << std::endl;
                                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                            }
                        }// end "telemetry" if
                    } else {
                        // Manual driving
                        string msg = "42[\"manual\",{}]";
                        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                    }
                }  // end websocket message if
            }); // end h.onMessage

    h.onConnection([&h](uWS::WebSocket <uWS::SERVER> ws, uWS::HttpRequest req) {
        std::cout << "Connected!!!" << std::endl;
    });

    h.onDisconnection([&h](uWS::WebSocket <uWS::SERVER> ws, int code,
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