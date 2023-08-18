#include "test_robot.hpp"
#include <memory>

class MockRobot : public Robot
{
public:
    // MOCK_METHOD(void, centerRobot, (int object_identifier), (override));
};

class ROBOT_TEST_SUITE : public ::testing::Test {
public:
    static void SetUpTestSuite() {
        rclcpp::init(0, nullptr);
    }

    static void TearDownTestSuite() {
        rclcpp::shutdown();
    }
};

TEST_F(ROBOT_TEST_SUITE, junebug_json)
{
    std::unique_ptr<Configuration> config = std::make_unique<Configuration>(JUNEBUG_CONFIG);
    std::shared_ptr<Robot> robot = RobotFactory::createRobot(*config);
    EXPECT_EQ(robot->name, "junebug");
    EXPECT_EQ(robot->motor_count, 2);
}

TEST_F(ROBOT_TEST_SUITE, percy_json)
{
    std::unique_ptr<Configuration> config = std::make_unique<Configuration>(PERCY_CONFIG);
    std::shared_ptr<Robot> robot = RobotFactory::createRobot(*config);
    EXPECT_EQ(robot->name, "percy");
    EXPECT_EQ(robot->motor_count, 8);
}

