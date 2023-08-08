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

TEST_F(ROBOT_TEST_SUITE, Midpoint) {
    std::unique_ptr<Robot> robot_ptr = std::make_unique<Robot>();
}
