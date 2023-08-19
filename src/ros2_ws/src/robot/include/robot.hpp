#pragma once
#include "robot_interface.hpp"
#include "configuration.hpp"
#include "can_interface.hpp"
#include "component.hpp"

class Robot : public rclcpp::Node
{
    public: 
        Robot(const Configuration& configuration);
        virtual void main_update_loop();
        std::string getName() const {return this->name;}
        int getMotorCount() const {return this->motor_count;}
        Interface::matrix_t getThrustMapper() const {return this->thrust_mapper;}
        std::shared_ptr<CanInterface::CanClient> getCanClient() const {return this->can_client;}
        const Configuration& getConfiguration() const {return this->configuration;}
        void connectComponents(std::shared_ptr<std::vector<std::shared_ptr<Component>>> components);
    protected:
        std::string name;
        int motor_count;
        Interface::matrix_t thrust_mapper;
        const Configuration& configuration;
        std::shared_ptr<CanInterface::CanClient> can_client;
        std::shared_ptr<std::vector<std::shared_ptr<Component>>> components;
    private:
        Interface::ros_timer_t update_timer;
};
