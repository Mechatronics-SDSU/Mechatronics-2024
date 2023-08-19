/* 
 * @author Conner Sommerfield - Zix on Discord
 * The infamous PID Controller Node
 * Subscribes to all sensor data needed for control system and throws it into a PID object
 * to output ctrl and thrust values.
 * Refer to classes/pid_controller for more info
 * 
 * Every time the subscription callback is triggered, the current state will be updated
 * Current state is stored in member variables of the Node
 * There is a timer that updates every x amount of ms based on what user inputs
 * This calls the PID update function which will use that sensor data to get ctrl/thrust values
 * Then it makes a CAN Request using that array of thrust_vals to send to motors
 * CAN Request is of ID #010
 */

#include "pid_node.hpp"

#define PID_ERROR_THRESHOLD 0.01f
#define MOTOR_ID 0x010
#define ENABLE_BYTE 0x00A

/////////////////////////////////////////////////////////////////////////////////////////////////////
                                    // MEMBER VARIABLE DECLARATIONS // 
/////////////////////////////////////////////////////////////////////////////////////////////////////

/** 
 * Controller Node consists of: 
 *      - timer to update ctrl_vals
 *      - subscription to desired state
 *      - subscription to ms5837 depth data
 *      - subscription to ZED position data
 *      - subscription to AHRS orientation data
 *      - Creates CAN Mailbox and ScionPIDController objects 
**/

Controller::Controller(): Node("pid_controller")
{
    current_state_sub_ = this->create_subscription<scion_types::msg::State>
    ("relative_current_state_data", 10, std::bind(&Controller::current_state_callback, this, std::placeholders::_1));

    controller_ = Scion_Position_PID_Controller(pid_params_object_.get_pid_params());
}

    /////////////////////////////////////////////////////////////////////////////////////////////////////
                                // WAIT FOR VALID DATA TO INITIALIZE PIDs // 
    /////////////////////////////////////////////////////////////////////////////////////////////////////

void Controller::initDesiredState()
{
    auto desiredValid = std::bind(&Controller::desiredStateValid, this);
    std::future<bool> promise = std::async(desiredValid);
    std::cout << "Waiting for Valid Sensor Info. \n";
    bool valid = promise.get();
    std::cout << "Got Valid Sensor Info. \n";
}

bool Controller::desiredStateValid()
{
    while (!this->current_state_valid_)
    {
        this->sendNothingAndWait();
        std::this_thread::sleep_for(std::chrono::milliseconds(SLEEP_TIME));
    }
    this->resetState();
    // std::this_thread::sleep_for(std::chrono::milliseconds(SLEEP_TIME));
    this->desired_state_ = vector<float>{0,0,0,0,0,0};
    while (!this->desired_state_valid_) {this->desired_state_valid_ = true;}
    while (!this->current_state_valid_) {this->current_state_valid_ = true;}
    canClient::sendFrame(ENABLE_BYTE, 0, nothing_.data(), can_client_);
    canClient::setBotInSafeMode(can_client_);
    this->pidReady();
    return true;
}

void Controller::printCurrentAndDesiredStates()
{
    std::cout << "DESIRED STATE: ";
    printVector(this->desired_state_);
    std::cout << "CURRENT STATE: ";
    printVector(this->current_state_);
}

void Controller::sendNothingAndWait()
{
    canClient::sendFrame(0x010, 8, nothing_.data(), can_client_); // Keep from exiting safe mode by sending 0 command
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
                                    // UPDATES OF STATE FOR PIDs //
/////////////////////////////////////////////////////////////////////////////////////////////////////


/* Different timer for printing and sensor updates */
void Controller::print_timer_callback()
{
    cout << "\n_________________________\n";
    this->printCurrentAndDesiredStates();
    cout << "___________________________\n\n";
    this->controller_.getStatus(); 
    stabilize_robot_ ? cout << "Stabilizing Robot" << " | " : cout << "Not Stabilizing Robot" << " | ";
    current_state_valid_ && desired_state_valid_ ? cout << "ROBOT IS UPDATING" << " | " : cout << "WAITING FOR GOOD SENSOR INFO" << " | ";
    use_position_ ? cout << "USING POSITION" << " | " : cout << "IGNORING POSITION" << endl;
}

/* Essential callback set to update PID state at certain interval */
void Controller::update_timer_callback()
{
    update_current_state();
}

void Controller::update_current_state() // ** Important Function **
{    
/* 
    * STEP 1: Update the PID Controller (meaning call the ScionPIDController object's
    * update function which generates ctrl_vals and show its status on the screen 
    */
    // sendNothingAndWait();
    if (stabilize_robot_ && current_state_valid_ && desired_state_valid_)
    {
        vector<float> thrusts(motor_count_, 0);
        thrusts = this->getThrusts(this->current_state_, this->desired_state_);
        make_CAN_request(thrusts);
    }
}

vector<float> Controller::getErrors(vector<float>& current_state, vector<float>& desired_state) 
{
    return desired_state - current_state;
}    

vector<float> Controller::getThrusts(vector<float>& current_state, vector<float>& desired_state)
{
    vector<float> errors                   {0.0F,0.0F,0.0F,0.0F,0.0F,0.0F}; 
    vector<float> adjustedErrors           {0.0F,0.0F,0.0F,0.0F,0.0F,0.0F};
    vector<float> ctrl_vals                {0.0F,0.0F,0.0F,0.0F,0.0F,0.0F}; 

    errors = getErrors(current_state,  desired_state);
    adjustedErrors = adjustErrors(errors);
    ctrl_vals = this->controller_.update(adjustedErrors, (float)UPDATE_PERIOD_RAW / 1000);
    return ctrlValsToThrusts(ctrl_vals);
}    

vector<float> Controller::ctrlValsToThrusts(vector<float>& ctrl_vals)
{
    return this->thrust_mapper_ * ctrl_vals;
}



vector<float> Controller::update_PID(Interface::current_state_t& current_state, Interface::desired_state_t& desired_state)
{
    using namespace Interface;
    if (!this->current_state_valid_ || !this->desired_state_valid_) 
    {
        return vector<float>(this->motor_count_, 0);
    }

    if (this->use_position_)
    {
        return this->getThrusts(current_state, desired_state); 
    }
    else
    {
        current_state_t current_state_no_position = current_state_t{current_state[0], current_state[1], current_state[2], 0, 0, 0};
        desired_state_t desired_state_no_position = desired_state_t{desired_state[0], desired_state[1], desired_state[2], 0, 0, 0};
        return this->getThrusts(current_state_no_position, desired_state_no_position);    
    }

}





void Controller::execute(const std::shared_ptr<GoalHandlePIDAction> goal_handle)
{
    /* Goal Initialization */
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    rclcpp::Rate loop_rate(20);

    std::shared_ptr<PIDAction::Feedback> feedback = std::make_shared<PIDAction::Feedback>();
    std::shared_ptr<PIDAction::Result> result = std::make_shared<PIDAction::Result>();
    const auto goal = goal_handle->get_goal();

    this->desired_state_ = goal->desired_state;

    this->stabilize_robot_ = false;
    /* Init States */
    std::vector<float>& state = feedback->current_state;
    vector<float> thrusts = this->update_PID(this->current_state_, this->desired_state_);
    vector<int> thrustInts = this->make_CAN_request(thrusts);
    for (int thrust : thrustInts)
    {
        state.push_back((float)thrust);
    }

    int cycles_at_set_point = 0;
    deque<vector<int>> slew_buffer;
    bool slew_rate_low = false;
    /* Feedback Loop - Stop Conditions is that all Thrusts are close to zero*/
    while (!this->equalToZero(thrustInts) && (cycles_at_set_point <= 5) && !slew_rate_low)     {
        //   Check if there is a cancel request
        if (goal_handle->is_canceling()) {
            goal_handle->canceled(result);
            RCLCPP_INFO(this->get_logger(), "Goal canceled");
            return;
        }
        this->areEqual(current_state_, desired_state_) ? cycles_at_set_point++ : cycles_at_set_point=0; 
        slew_buffer.push_front(thrustInts);
            if (slew_buffer.size() > 10) {
                slew_buffer.pop_back();
                slew_rate_low = isSlewRateLow(calculateTotalSlew(slew_buffer));
            }
        /* Update at Every Loop */
        thrusts = update_PID(this->current_state_, this->desired_state_);
        thrustInts = this->make_CAN_request(thrusts);
        for (int i = 0; i < thrustInts.size(); i++)
        { 
            state[i] = ((float)thrustInts[i]);
        }

        goal_handle->publish_feedback(feedback);
        loop_rate.sleep();
    }
    this->stabilize_robot_ = true;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
                                        // CONTROL SERVICES // 
/////////////////////////////////////////////////////////////////////////////////////////////////////

void Controller::stopRobot(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    this->desired_state_ = this->current_state_;
}

void Controller::pidReady()
{
    auto pid_ready_request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto pid_ready_future = this->pid_ready_client_->async_send_request(pid_ready_request);
}

void Controller::resetState()
{
    auto reset_state_request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto reset_state_future = this->reset_relative_state_client_->async_send_request(reset_state_request);
}

void Controller::resetPosition()
{     
    auto reset_position_request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto reset_position_future = this->reset_relative_position_client_->async_send_request(reset_position_request);
}

void Controller::usePosition(const  std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                        std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
    this->use_position_ = request->data;
}

void Controller::stabilizeRobot(const   std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                            std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
    this->stabilize_robot_ = request->data;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
                                    // SUBSCRIPTION CALLBACKS // 
/////////////////////////////////////////////////////////////////////////////////////////////////////

void Controller::current_state_callback(const scion_types::msg::State::SharedPtr msg)
{
    if (!this->current_state_valid_) {this->current_state_valid_ = true;}
    this->current_state_= msg->state; 
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
                                    // GUI TUNING CONTROLS CALLBACKS // 
/////////////////////////////////////////////////////////////////////////////////////////////////////

void Controller::kp_tuning_callback(const scion_types::msg::PidTuning::SharedPtr msg) 
{
    shared_ptr<PID_Controller> axis = this->controller_.controllers[axis_tuning_map_[msg->axis]];
    // float scale_factor = axis->angle_wrap ? .0001 : .001;
    // axis->set_gains(scale_factor * msg->data, axis->k_i, axis->k_d);

    axis->set_gains(msg->data, axis->k_i, axis->k_d);
}

void Controller::ki_tuning_callback(const scion_types::msg::PidTuning::SharedPtr msg) 
{
    shared_ptr<PID_Controller> axis = this->controller_.controllers[axis_tuning_map_[msg->axis]];
    // float scale_factor = axis->angle_wrap ? .0001 : .001;
    // axis->set_gains(axis->k_p, scale_factor * msg->data, axis->k_d);

    axis->set_gains(axis->k_i, msg->data, axis->k_d);
}

void Controller::kd_tuning_callback(const scion_types::msg::PidTuning::SharedPtr msg) 
{
    shared_ptr<PID_Controller> axis = this->controller_.controllers[axis_tuning_map_[msg->axis]];
    // float scale_factor = axis->angle_wrap ? .0001 : .001;
    // axis->set_gains(axis->k_p, axis->k_i, scale_factor * msg->data);

    axis->set_gains(axis->k_d, axis->k_i, msg->data);
}