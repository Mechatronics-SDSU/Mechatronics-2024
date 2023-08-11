#pragma once
#include "robot_interface.hpp"
#include "abstract_robot.hpp"
#include "configuration.hpp"
#include "gui_listener_node.hpp"
#include "controller_node.hpp"

#define protected public 
class Robot : public AbstractRobot, public rclcpp::Node
{
    public: 
        Robot();
        virtual void main_update_loop() override;
        std::string getName() override {return this->name;}
        int getMotorCount() override {return this->motor_count;}
        Interface::matrix_t getThrustMapper() override {return this->thrust_mapper;}
        std::shared_ptr<CanInterface::CanClient> getCanClient() override {return this->can_client;}
    protected:
        std::string name;
        int motor_count;
        Interface::matrix_t thrust_mapper;
        std::shared_ptr<CanInterface::CanClient> can_client;
    private:
        Interface::ros_timer_t update_timer;
        std::shared_ptr<GUI_Listener> gui_listener;
        std::shared_ptr<Controller> controller_node;
};
