#pragma once
#include "robot_interface.hpp"
#include "configuration.hpp"
#include "can_interface.hpp"

class Robot : public rclcpp::Node
{
    public: 
        Robot(const Configuration& configuration);
        virtual void main_update_loop();
        std::string getName() {return this->name;}
        int getMotorCount() {return this->motor_count;}
        Interface::matrix_t getThrustMapper() {return this->thrust_mapper;}
        std::shared_ptr<CanInterface::CanClient> getCanClient() {return this->can_client;}
        const Configuration& getConfiguration() const {return this->configuration;}
    protected:
        std::string name;
        int motor_count;
        Interface::matrix_t thrust_mapper;
        std::shared_ptr<CanInterface::CanClient> can_client;
        const Configuration& configuration;
    private:
        Interface::ros_timer_t update_timer;
};
