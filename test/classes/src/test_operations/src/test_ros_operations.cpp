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


TEST_F(OPERATIONS_TEST_SUITE, test_ros_operations)
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
