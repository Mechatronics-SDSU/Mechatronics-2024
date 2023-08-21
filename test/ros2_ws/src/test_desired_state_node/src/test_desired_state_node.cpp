#include "robot.hpp"
#include "robot_interface.hpp"
#include "gtest/gtest.h"
#include "gmock/gmock.h"
#include "can_interface.hpp"
#include "components.hpp"
#include "component.hpp"
#include "desired_state_node.hpp"
#include <chrono>

class DESIRED_STATE_NODE_TEST_SUITE : public ::testing::Test {
public:
    static void SetUpTestSuite() {
        rclcpp::init(0, nullptr);
    }

    static void TearDownTestSuite() {
        rclcpp::shutdown();
    }
};

namespace
{
    std::string waitForData()
    {
        std::string message = "";
        std::promise<bool> got_string;
        std::shared_future<bool> future  = got_string.get_future();
        Interface::node_t temp_node = rclcpp::Node::make_shared("desired_state_node_subscriber");
        Interface::string_sub_t string_sub = temp_node->create_subscription<std_msgs::msg::String>("desired_state_data", 10, [&message, &got_string](const std_msgs::msg::String::SharedPtr msg) {
            message = msg->data;
            got_string.set_value(true);
        });
        rclcpp::spin_until_future_complete(temp_node, future);
        return message;
    }
}

TEST_F(DESIRED_STATE_NODE_TEST_SUITE, test_desired_state_node_subscription)
{
    Interface::node_t temp_node = rclcpp::Node::make_shared("desired_state_node_publisher");
    Interface::string_pub_t desired_state_node_pub = temp_node->create_publisher<std_msgs::msg::String>("desired_state_data", 10);
    Interface::ros_timer_t desired_state_node_timer = temp_node->create_wall_timer(std::chrono::milliseconds(10), [&desired_state_node_pub]() {
        std_msgs::msg::String msg = std_msgs::msg::String();
        msg.data = "Zix was here";
        desired_state_node_pub->publish(msg);
    });   

    std::shared_future<std::string> string_message = std::async(std::launch::async, &waitForData);
    rclcpp::spin_until_future_complete(temp_node, string_message);
    EXPECT_EQ(string_message.get(), "Zix was here");
}
