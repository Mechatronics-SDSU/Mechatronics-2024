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
    void spinNode(const Interface::node_t& node, const std::shared_future<std::shared_ptr<scion_types::srv::GetDesiredState::Response>>& future)
    {
        rclcpp::spin_until_future_complete(node, future);
    }
}

TEST_F(DESIRED_STATE_NODE_TEST_SUITE, test_get_desired_state_node_subscription)
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

namespace
{
    scion_types::msg::State makeRobotState()
    {
        scion_types::msg::State state = scion_types::msg::State();
        state.orientation.yaw.value = 10;
        state.orientation.yaw.set = true;
        state.orientation.pitch.value = 20;
        state.orientation.pitch.set = true;
        state.orientation.roll.value = 30;
        state.orientation.roll.set = true;
        return state;
    }
}

TEST_F(DESIRED_STATE_NODE_TEST_SUITE, test_change_desired_state_node_subscription)
{
    std::shared_ptr<DesiredStateNode> desired_node_ptr = std::make_shared<DesiredStateNode>();
    
    Interface::node_t temp_node = rclcpp::Node::make_shared("desired_state_node_client");
    Interface::change_desired_state_client_t change_desired_state_node_client =  temp_node->create_client<scion_types::srv::ChangeDesiredState>("change_desired_state");
    
    auto change_desired_state_request = std::make_shared<scion_types::srv::ChangeDesiredState::Request>();
    change_desired_state_request->requester_name = "Zix & Friends";
    change_desired_state_request->requested_state = makeRobotState();
    auto future = change_desired_state_node_client->async_send_request(change_desired_state_request);
    
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
    rclcpp::spin_some(desired_node_ptr);
    rclcpp::spin_some(temp_node);
    rclcpp::spin_some(desired_node_ptr);
    rclcpp::spin_some(temp_node);

    auto response = future.get();
    
    EXPECT_EQ(response->desired_state.orientation.yaw.value, 10);
    EXPECT_EQ(response->desired_state.orientation.pitch.value, 20);
    EXPECT_EQ(response->desired_state.orientation.roll.value, 30);
    EXPECT_EQ(response->desired_state.position.x_pos.value, 0);
    EXPECT_EQ(response->desired_state.position.y_pos.value, 0);
    EXPECT_EQ(response->desired_state.position.z_pos.value, 0);
}


/*
        // auto state_message2 = std::async(std::launch::async, [&temp_node, &future]() {
        //     rclcpp::spin_until_future_complete(temp_node, future);
        // });
        // auto state_message = std::async(std::launch::async, [&desired_node_ptr, &future]() {
        //     spinNode(desired_node_ptr, future);
        // });
*/