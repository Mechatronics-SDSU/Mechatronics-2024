#ifndef A50_H
#define A50_H

#include <vector>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <nlohmann/json.hpp>

#include "robot_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "vector_operations.hpp"

class A50Node : public rclcpp::Node
{
public:
    explicit A50Node();

private:
    int sock_ = 0;
    struct sockaddr_in serv_addr_;
    Interface::state_pub_t position_publisher_;
    Interface::state_pub_t orientation_publisher_;
    Interface::ros_timer_t get_data_timer_;

    void getA50Data();
    void publishData(std::vector<float>& a50_data);
};

#endif // A50_H