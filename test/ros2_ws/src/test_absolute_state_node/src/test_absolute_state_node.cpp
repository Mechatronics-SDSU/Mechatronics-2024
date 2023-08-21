#include "robot.hpp"
#include "robot_interface.hpp"
#include "gtest/gtest.h"
#include "gmock/gmock.h"
#include "can_interface.hpp"
#include "components.hpp"
#include "component.hpp"
#include "absolute_state_node.hpp"
#include "robot_interface.hpp"
#include <chrono>

class ABSOLUTE_STATE_NODE_TEST_SUITE : public ::testing::Test {
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
    scion_types::msg::State::SharedPtr waitForData()
    {
        scion_types::msg::State::SharedPtr message;
        std::promise<bool> got_message;
        std::shared_future<bool> future  = got_message.get_future();
        Interface::node_t temp_node = rclcpp::Node::make_shared("absolute_state_node_subscriber");
        Interface::state_sub_t state_sub = temp_node->create_subscription<scion_types::msg::State>("absolute_state_data", 10, [&message, &got_message](const scion_types::msg::State::SharedPtr msg) {
            message = msg;
            got_message.set_value(true);
        });
        rclcpp::spin_until_future_complete(temp_node, future);
        return message;
    }
}

namespace
{
    scion_types::msg::State createOrientationStateMessage()
    {
        scion_types::msg::State msg = scion_types::msg::State();
        msg.orientation.yaw.value = 10;
        msg.orientation.yaw.set = true;
        msg.orientation.pitch.value = 20;
        msg.orientation.pitch.set = true;
        msg.orientation.roll.value = 30;
        msg.orientation.roll.set = true;
        msg.position.x_pos.value = 30;
        msg.position.x_pos.set = false;
        msg.position.y_pos.value = 20;
        msg.position.y_pos.set = false;
        msg.position.z_pos.value = 10;
        msg.position.z_pos.set = false;
        return msg;   
    }   

    scion_types::msg::State createPositionStateMessage()
    {
        scion_types::msg::State msg = scion_types::msg::State();
        msg.position.x_pos.value = 30;
        msg.position.x_pos.set = true;
        msg.position.y_pos.value = 20;
        msg.position.y_pos.set = true;
        msg.position.z_pos.value = 10;
        msg.position.z_pos.set = true;
        return msg;   
    }   
}


TEST_F(ABSOLUTE_STATE_NODE_TEST_SUITE, test_ahrs_state_node_subscription)
{
    std::shared_ptr<AbsoluteStateNode> state_node = std::make_shared<AbsoluteStateNode>();
    Interface::node_t temp_node = rclcpp::Node::make_shared("absolute_state_node_publisher");
    Interface::state_pub_t absolute_state_node_pub = temp_node->create_publisher<scion_types::msg::State>("ahrs_state_data", 10);
    Interface::ros_timer_t absolute_state_node_timer = temp_node->create_wall_timer(std::chrono::milliseconds(10), [&state_node, &absolute_state_node_pub]() {
        scion_types::msg::State msg = createOrientationStateMessage();
        rclcpp::spin_some(state_node);
        absolute_state_node_pub->publish(msg);
    });   

    std::shared_future<scion_types::msg::State::SharedPtr> state_message = std::async(std::launch::async, &waitForData);
    rclcpp::spin_until_future_complete(temp_node, state_message);
    scion_types::msg::State::SharedPtr msg = state_message.get();
    EXPECT_EQ(msg->orientation.yaw.value,   10);
    EXPECT_EQ(msg->orientation.pitch.value, 20);
    EXPECT_EQ(msg->orientation.roll.value,  30);
    EXPECT_NEAR(msg->position.x_pos.value,  0, .01);
    EXPECT_NEAR(msg->position.y_pos.value,  0, .01);
    EXPECT_NEAR(msg->position.z_pos.value,  0, .01);
}

TEST_F(ABSOLUTE_STATE_NODE_TEST_SUITE, test_a50_state_node_subscription)
{
    std::shared_ptr<AbsoluteStateNode> state_node = std::make_shared<AbsoluteStateNode>();
    Interface::node_t temp_node = rclcpp::Node::make_shared("absolute_state_node_publisher");
    Interface::state_pub_t absolute_state_node_pub = temp_node->create_publisher<scion_types::msg::State>("a50_state_data", 10);
    Interface::ros_timer_t absolute_state_node_timer = temp_node->create_wall_timer(std::chrono::milliseconds(10), [&state_node, &absolute_state_node_pub]() {
        scion_types::msg::State msg = createPositionStateMessage();
        rclcpp::spin_some(state_node);
        absolute_state_node_pub->publish(msg);
    });   

    std::shared_future<scion_types::msg::State::SharedPtr> state_message = std::async(std::launch::async, &waitForData);
    rclcpp::spin_until_future_complete(temp_node, state_message);
    scion_types::msg::State::SharedPtr msg = state_message.get();
    EXPECT_EQ(msg->orientation.yaw.value,   0);
    EXPECT_EQ(msg->orientation.pitch.value, 0);
    EXPECT_EQ(msg->orientation.roll.value,  0);
    EXPECT_NEAR(msg->position.x_pos.value,  30, .01);
    EXPECT_NEAR(msg->position.y_pos.value,  20, .01);
    EXPECT_NEAR(msg->position.z_pos.value,  10, .01);
}

TEST_F(ABSOLUTE_STATE_NODE_TEST_SUITE, test_integrated_nodes_subscription)
{
    std::shared_ptr<AbsoluteStateNode> state_node = std::make_shared<AbsoluteStateNode>();

    Interface::node_t temp_node = rclcpp::Node::make_shared("absolute_state_node_publisher");
    Interface::state_pub_t absolute_state_node_pub = temp_node->create_publisher<scion_types::msg::State>("ahrs_state_data", 10);
    Interface::ros_timer_t absolute_state_node_timer = temp_node->create_wall_timer(std::chrono::milliseconds(10), [&state_node, &absolute_state_node_pub]() {
        scion_types::msg::State msg = createOrientationStateMessage();
        rclcpp::spin_some(state_node);
        absolute_state_node_pub->publish(msg);
    });   

    Interface::node_t temp_node2 = rclcpp::Node::make_shared("absolute_state_node_publisher2");
    Interface::state_pub_t absolute_state_node_pub2 = temp_node2->create_publisher<scion_types::msg::State>("a50_state_data", 10);
    Interface::ros_timer_t absolute_state_node_timer2 = temp_node2->create_wall_timer(std::chrono::milliseconds(10), [&state_node, &absolute_state_node_pub2]() {
        scion_types::msg::State msg = createPositionStateMessage();
        rclcpp::spin_some(state_node);
        absolute_state_node_pub2->publish(msg);
    });   

    std::shared_future<scion_types::msg::State::SharedPtr> state_message = std::async(std::launch::async, &waitForData);
    rclcpp::spin_until_future_complete(temp_node, state_message);
    std::shared_future<scion_types::msg::State::SharedPtr> state_message2 = std::async(std::launch::async, &waitForData);
    rclcpp::spin_until_future_complete(temp_node2, state_message2);

    scion_types::msg::State::SharedPtr msg = state_message2.get();
    EXPECT_EQ(msg->orientation.yaw.value,   10);
    EXPECT_EQ(msg->orientation.pitch.value, 20);
    EXPECT_EQ(msg->orientation.roll.value,  30);
    EXPECT_NEAR(msg->position.x_pos.value,  30, .01);
    EXPECT_NEAR(msg->position.y_pos.value,  20, .01);
    EXPECT_NEAR(msg->position.z_pos.value,  10, .01);
}
