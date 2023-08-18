#include "test_robot.hpp"
#include <memory>

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
    std::unique_ptr<Configuration> config = std::make_unique<Configuration>(JUNEBUG_CONFIG); // Declared in CMakeLists.txt
    std::shared_ptr<Robot> robot = RobotFactory::createRobot(*config);
    EXPECT_EQ(robot->getName(), "junebug");
    EXPECT_EQ(robot->getMotorCount(), 2);
}

TEST_F(ROBOT_TEST_SUITE, percy_json)
{
    std::unique_ptr<Configuration> config = std::make_unique<Configuration>(PERCY_CONFIG); // Declared in CMakeLists.txt
    std::shared_ptr<Robot> robot = RobotFactory::createRobot(*config);
    EXPECT_EQ(robot->getName(), "percy");
    EXPECT_EQ(robot->getMotorCount(), 8);
}

TEST_F(ROBOT_TEST_SUITE, config_move)
{
    std::unique_ptr<Configuration> config = std::make_unique<Configuration>(JUNEBUG_CONFIG); // Declared in CMakeLists.txt
    std::shared_ptr<Robot> robot = RobotFactory::createRobot(*config);
    EXPECT_EQ(robot->getConfiguration().getJsonString()["robot"], "junebug");
}