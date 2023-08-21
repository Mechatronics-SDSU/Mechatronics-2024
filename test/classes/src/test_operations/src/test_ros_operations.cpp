#include "gtest/gtest.h"
#include "gmock/gmock.h"
#include "robot_interface.hpp"
#include "ros_operations.hpp"
#include <chrono>

class OPERATIONS_TEST_SUITE : public ::testing::Test {
public:
    static void SetUpTestSuite() {
    }

    static void TearDownTestSuite() {
    }
};

namespace
{
    scion_types::msg::State::SharedPtr createStateMessage()
    {
        const scion_types::msg::State::SharedPtr msg = std::make_shared<scion_types::msg::State>();
        msg->orientation.yaw.value = 10;
        msg->orientation.yaw.set = true;
        msg->orientation.pitch.value = 20;
        msg->orientation.pitch.set = true;
        msg->orientation.roll.value = 30;
        msg->orientation.roll.set = true;
        msg->position.x_pos.value = 30;
        msg->position.x_pos.set = false;
        msg->position.y_pos.value = 20;
        msg->position.y_pos.set = false;
        msg->position.z_pos.value = 10;
        msg->position.z_pos.set = false;
        return msg;   
    }   
}

TEST_F(OPERATIONS_TEST_SUITE, test_message)
{
    const scion_types::msg::State::SharedPtr msg = createStateMessage();
    EXPECT_EQ(msg->orientation.yaw.value,   10);
    EXPECT_EQ(msg->orientation.pitch.value, 20);
    EXPECT_EQ(msg->orientation.roll.value,  30);
    EXPECT_NEAR(msg->position.x_pos.value,  30, .01);
    EXPECT_NEAR(msg->position.y_pos.value,  20, .01);
    EXPECT_NEAR(msg->position.z_pos.value,  10, .01);
}


TEST_F(OPERATIONS_TEST_SUITE, test_copy_msg_to_state)
{
    const scion_types::msg::State::SharedPtr msg = createStateMessage();
    Interface::RobotState robot_state;
    rosOperations::copyRobotState(msg, robot_state);

    EXPECT_EQ(robot_state.orientation.yaw,   10);
    EXPECT_EQ(robot_state.orientation.pitch, 20);
    EXPECT_EQ(robot_state.orientation.roll,  30);
    EXPECT_NEAR(robot_state.position.x_pos,  0, .01);
    EXPECT_NEAR(robot_state.position.y_pos,  0, .01);
    EXPECT_NEAR(robot_state.position.z_pos,  0, .01);
}

namespace
{
    scion_types::srv::ChangeDesiredState::Request::SharedPtr createStateRequest()
    {
        scion_types::srv::ChangeDesiredState::Request::SharedPtr request = std::make_shared<scion_types::srv::ChangeDesiredState::Request>();
        scion_types::msg::State state = scion_types::msg::State();

        state.orientation.yaw.value = 10;
        state.orientation.yaw.set = true;
        state.orientation.pitch.value = 20;
        state.orientation.pitch.set = true;
        state.orientation.roll.value = 30;
        state.orientation.roll.set = true;
        state.position.x_pos.value = 30;
        state.position.x_pos.set = false;
        state.position.y_pos.value = 20;
        state.position.y_pos.set = false;
        state.position.z_pos.value = 10;
        state.position.z_pos.set = false;

        request->requested_state = state;

        return request;   
    }   
}

TEST_F(OPERATIONS_TEST_SUITE, test_copy_srv_to_state)
{
    scion_types::srv::ChangeDesiredState::Request::SharedPtr request = createStateRequest();
    Interface::RobotState robot_state;
    rosOperations::copyRobotState(request, robot_state);

    EXPECT_EQ(robot_state.orientation.yaw,   10);
    EXPECT_EQ(robot_state.orientation.pitch, 20);
    EXPECT_EQ(robot_state.orientation.roll,  30);
    EXPECT_NEAR(robot_state.position.x_pos,  0, .01);
    EXPECT_NEAR(robot_state.position.y_pos,  0, .01);
    EXPECT_NEAR(robot_state.position.z_pos,  0, .01);
}

namespace
{
    Interface::RobotState createState()
    {
        Interface::RobotState state;

        state.orientation.yaw = 10;
        state.orientation.pitch = 20;
        state.orientation.roll = 30;
        state.position.x_pos = 30;
        state.position.y_pos = 20;
        state.position.z_pos = 10;

        return state;   
    }   
}

TEST_F(OPERATIONS_TEST_SUITE, test_state_to_change_response)
{
    Interface::RobotState robot_state = createState();
    scion_types::srv::ChangeDesiredState::Response response = scion_types::srv::ChangeDesiredState::Response();
    rosOperations::copyRobotState(robot_state, response);

    EXPECT_EQ(response.desired_state.orientation.yaw.value,   10);
    EXPECT_EQ(response.desired_state.orientation.pitch.value, 20);
    EXPECT_EQ(response.desired_state.orientation.roll.value,  30);
    EXPECT_EQ(response.desired_state.position.x_pos.value,    30);
    EXPECT_EQ(response.desired_state.position.y_pos.value,    20);
    EXPECT_EQ(response.desired_state.position.z_pos.value,    10);
}

TEST_F(OPERATIONS_TEST_SUITE, test_state_to_get_response)
{
    Interface::RobotState robot_state = createState();
    scion_types::srv::GetDesiredState::Response response = scion_types::srv::GetDesiredState::Response();
    rosOperations::copyRobotState(robot_state, response);

    EXPECT_EQ(response.desired_state.orientation.yaw.value,   10);
    EXPECT_EQ(response.desired_state.orientation.pitch.value, 20);
    EXPECT_EQ(response.desired_state.orientation.roll.value,  30);
    EXPECT_EQ(response.desired_state.position.x_pos.value,    30);
    EXPECT_EQ(response.desired_state.position.y_pos.value,    20);
    EXPECT_EQ(response.desired_state.position.z_pos.value,    10);
}