#include "robot.hpp"
#include "robot_factory.hpp"
#include "gtest/gtest.h"
#include "gmock/gmock.h"
#include "can_interface.hpp"
#include "components.hpp"
#include "component.hpp"

class ROBOT_TEST_SUITE : public ::testing::Test {
public:
    static void SetUpTestSuite() {
        rclcpp::init(0, nullptr);
    }

    static void TearDownTestSuite() {
        rclcpp::shutdown();
    }
};

TEST_F(ROBOT_TEST_SUITE, create_component_vector)
{
    std::unique_ptr<Configuration> config = std::make_unique<Configuration>(JUNEBUG_CONFIG); // Declared in CMakeLists.txt
    std::shared_ptr<Robot> robot = RobotFactory::createRobot(*config);
    std::vector<std::string> nodes_to_enable = robot->getConfiguration().getJsonString()["nodes"]; 
    EXPECT_EQ(robot->getConfiguration().getJsonString()["nodes"][0], "controller_node");
    EXPECT_EQ(robot->getConfiguration().getJsonString()["nodes"][1], "gui_listener");
    EXPECT_EQ(nodes_to_enable[0], "controller_node");
    EXPECT_EQ(nodes_to_enable[1], "gui_listener");
}

TEST_F(ROBOT_TEST_SUITE, create_components)
{
    std::unique_ptr<Configuration> config = std::make_unique<Configuration>(JUNEBUG_CONFIG); // Declared in CMakeLists.txt
    std::shared_ptr<Robot> robot = RobotFactory::createRobot(*config);
    std::shared_ptr<std::vector<std::shared_ptr<Component>>> components = Components::CreateComponentVector(*robot);
    EXPECT_EQ(components->size(), 2);
}
