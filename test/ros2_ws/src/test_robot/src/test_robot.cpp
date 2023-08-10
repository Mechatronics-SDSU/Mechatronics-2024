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
    RobotFactory::registerRobots();
    std::unique_ptr<Configuration> config = std::make_unique<Configuration>(JUNEBUG_CONFIG);
    nlohmann::json json_string = config->getJsonString();

    RobotType type = RobotFactory::getType(json_string["robot"]);
    std::shared_ptr<Robot> robot = RobotFactory::CreateRobot(type, *config);
    EXPECT_EQ(robot->name, "junebug");
    EXPECT_EQ(robot->motor_count, 2);
}

TEST_F(ROBOT_TEST_SUITE, percy_json)
{
    RobotFactory::registerRobots();
    std::unique_ptr<Configuration> config = std::make_unique<Configuration>(PERCY_CONFIG);
    nlohmann::json json_string = config->getJsonString();

    RobotType type = RobotFactory::getType(json_string["robot"]);
    std::shared_ptr<Robot> robot = RobotFactory::CreateRobot(type, *config);
    EXPECT_EQ(robot->name, "percy");
    EXPECT_EQ(robot->motor_count, 8);
}

TEST_F(ROBOT_TEST_SUITE, get_type)
{
    RobotType actualType = RobotFactory::getType("junebug");
    RobotType expectedType = RobotType::Junebug;
    EXPECT_EQ(expectedType, actualType);
}

TEST_F(ROBOT_TEST_SUITE, get_invalid_type)
{
    RobotType actualType = RobotFactory::getType("oogabalooga");
    RobotType expectedType = RobotType::Percy;
    EXPECT_EQ(expectedType, actualType);
}

