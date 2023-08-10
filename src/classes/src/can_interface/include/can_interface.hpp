#include "robot_interface.hpp"

class canClient
{
    public:
        void sendFrame(int32_t can_id, int8_t can_dlc, unsigned char can_data[]);
        void setBotInSafeMode();
        void turnOnLight();
        void turnOffLight();
        void killRobot();
        void allClear();
        std::vector<int> make_motor_request(std::vector<float>& thrusts, int motor_count, int max_power);
    private:
        Interface::node_t node;
        Interface::ros_sendframe_client_t can_client;
};