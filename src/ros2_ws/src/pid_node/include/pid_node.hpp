#ifndef PID_H
#define PID_H

#include <memory>
#include <string>
#include <ctime>
#include <thread>
#include <unistd.h>
#include <iostream>
#include <chrono>
#include <iomanip>
#include <vector>
#include <cstring>
#include <cmath>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "control_interface.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "scion_types/action/pid.hpp"
#include "scion_types/msg/state.hpp"
#include "scion_pid_controller.hpp"                // PID Class
#include "pid_controller.hpp"   

/////////////////////////////////////////////////////////////////////////////////////////////////////
                                    // MEMBER VARIABLE DECLARATIONS // 
/////////////////////////////////////////////////////////////////////////////////////////////////////

class Controller : public rclcpp::Node
{
public:
    explicit Controller();

// private:
    Interface::state_sub_t                      current_state_sub_;
    Interface::state_sub_t                      desired_state_sub_;
    Scion_Position_PID_Controller               controller_;
    PID_Params                                  pid_params_object_;                      // Passed to controller for tuning
     
    /* Upon initialization set all values to [0,0,0] */
    
    Interface::current_state_t current_state_{0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F}; // State described by yaw, pitch, roll, x, y, z 
    Interface::desired_state_t desired_state_{0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F}; // Desired state is that everything is set to 0 except that its 1 meter below the water {0,0,0,0,0,1}

    /////////////////////////////////////////////////////////////////////////////////////////////////////
                                // WAIT FOR VALID DATA TO INITIALIZE PIDs // 
    /////////////////////////////////////////////////////////////////////////////////////////////////////

    void initDesiredState();
    bool desiredStateValid();
    void printCurrentAndDesiredStates();
    void sendNothingAndWait();

    /////////////////////////////////////////////////////////////////////////////////////////////////////
                                        // UPDATES OF STATE FOR PIDs //
    /////////////////////////////////////////////////////////////////////////////////////////////////////

    void print_timer_callback();
    void update_timer_callback();

    void update_current_state();
    std::vector<float> getErrors(std::vector<float>& current_state, std::vector<float>& desired_state) ;
    std::vector<float> getThrusts(std::vector<float>& current_state, std::vector<float>& desired_state);
    std::vector<float> ctrlValsToThrusts(std::vector<float>& ctrl_vals);
    float angleBetweenHereAndPoint(float x, float y);
    float distanceBetweenHereAndPoint(float x, float y);
    std::vector<float> adjustErrors(std::vector<float>& errors);
    std::vector<float> update_PID(Interface::current_state_t& current_state, Interface::desired_state_t& desired_state);

    /////////////////////////////////////////////////////////////////////////////////////////////////////
                                            // CONTROL SERVICES // 
    /////////////////////////////////////////////////////////////////////////////////////////////////////

    void stopRobot(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    void pidReady();
    void resetState();
    void resetPosition();
    void usePosition(const  std::shared_ptr<std_srvs::srv::SetBool::Request> request, std::shared_ptr<std_srvs::srv::SetBool::Response> response);
    void stabilizeRobot(const std::shared_ptr<std_srvs::srv::SetBool::Request> request, std::shared_ptr<std_srvs::srv::SetBool::Response> response);

    /////////////////////////////////////////////////////////////////////////////////////////////////////
                                        // SUBSCRIPTION CALLBACKS // 
    /////////////////////////////////////////////////////////////////////////////////////////////////////
    
    void current_state_callback(const scion_types::msg::State::SharedPtr msg);

    /////////////////////////////////////////////////////////////////////////////////////////////////////
                                        // GUI TUNING CONTROLS CALLBACKS // 
    /////////////////////////////////////////////////////////////////////////////////////////////////////

    void kp_tuning_callback(const scion_types::msg::PidTuning::SharedPtr msg);
    void ki_tuning_callback(const scion_types::msg::PidTuning::SharedPtr msg);
    void kd_tuning_callback(const scion_types::msg::PidTuning::SharedPtr msg);
};

#endif