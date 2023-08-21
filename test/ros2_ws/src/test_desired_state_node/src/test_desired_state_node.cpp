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

}

TEST_F(DESIRED_STATE_NODE_TEST_SUITE, test_desired_state_node_subscription)
{
    std::shared_ptr<DesiredStateNode> desired_node_ptr = std::make_shared<DesiredStateNode>();
    
    Interface::node_t temp_node = rclcpp::Node::make_shared("desired_state_node_client");
    Interface::get_desired_state_client_t get_desired_state_node_client =  temp_node->create_client<scion_types::srv::GetDesiredState>("get_desired_state");
    
    auto get_desired_state_request = std::make_shared<scion_types::srv::GetDesiredState::Request>();
    get_desired_state_request->requester_name = "Zix & Friends";
    auto future = get_desired_state_node_client->async_send_request(get_desired_state_request);
    rclcpp::spin_some(desired_node_ptr);
    rclcpp::spin_some(temp_node);
    rclcpp::spin_some(desired_node_ptr);
    rclcpp::spin_some(temp_node);
    rclcpp::spin_some(desired_node_ptr);
    rclcpp::spin_some(temp_node);
    rclcpp::spin_some(desired_node_ptr);
    rclcpp::spin_some(temp_node);
    rclcpp::spin_some(desired_node_ptr);
    rclcpp::spin_some(temp_node);
    rclcpp::spin_some(desired_node_ptr);
    rclcpp::spin_some(temp_node);
    rclcpp::spin_some(desired_node_ptr);
    rclcpp::spin_some(temp_node);
    auto response = future.get();
    EXPECT_EQ(response->desired_state.orientation.yaw.value, 0);
    EXPECT_EQ(response->desired_state.orientation.pitch.value, 0);
    EXPECT_EQ(response->desired_state.orientation.roll.value, 0);
    EXPECT_EQ(response->desired_state.position.x_pos.value, 0);
    EXPECT_EQ(response->desired_state.position.y_pos.value, 0);
    EXPECT_EQ(response->desired_state.position.z_pos.value, 0);
}
