#include "robot_interface.hpp"

class canClient
{
    public:
        static void sendFrame(int32_t can_id, int8_t can_dlc, unsigned char can_data[]);
        static void setBotInSafeMode();
        static void turnOnLight();
        static void turnOffLight();
        static void killRobot();
        static void allClear();
        static std::vector<int> make_motor_request(std::vector<float>& thrusts, int motor_count, int max_power);
    private:
        static Interface::node_t node;
        static Interface::ros_sendframe_client_t can_client;
};