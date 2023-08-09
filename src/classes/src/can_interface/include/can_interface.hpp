#include "robot_interface.hpp"

namespace canClient
{
    void sendFrame(int32_t can_id, int8_t can_dlc, unsigned char can_data[], Interface::ros_sendframe_client_t can_client);
    void setBotInSafeMode(Interface::ros_sendframe_client_t can_client);
    void turnOnLight(Interface::ros_sendframe_client_t can_client);
    void turnOffLight(Interface::ros_sendframe_client_t can_client);
    void killRobot(Interface::ros_sendframe_client_t can_client);
    void allClear(Interface::ros_sendframe_client_t can_client);
    std::vector<int> make_CAN_request(std::vector<float>& thrusts, int motor_count, int max_power, Interface::ros_sendframe_client_t can_client);
}