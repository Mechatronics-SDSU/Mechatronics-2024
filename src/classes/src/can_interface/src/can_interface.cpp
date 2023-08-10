#include "can_interface.hpp"

Interface::node_t canClient::node = rclcpp::Node::make_shared("can_client");
Interface::ros_sendframe_client_t canClient::can_client = node->create_client<scion_types::srv::SendFrame>("send_can_raw");

void canClient::sendFrame(int32_t can_id, int8_t can_dlc, unsigned char can_data[])
{
    auto can_request = std::make_shared<scion_types::srv::SendFrame::Request>();
    can_request->can_id = can_id;
    can_request->can_dlc = can_dlc;
    std::copy
    (
        can_data,
        can_data + can_dlc,
        can_request->can_data.begin()
    );
    auto can_future = can_client->async_send_request(can_request);
    rclcpp::spin_until_future_complete(node, can_future);
}

void canClient::setBotInSafeMode()
{
    std::vector<unsigned char> safeModeFrame{0,0,0,0,0x04};
    sendFrame(0x022, 5, safeModeFrame.data());
}

void canClient::turnOnLight() 
{
    std::vector<unsigned char> lightEnable{0x04, 0x00, 0x00, 0x00, 0x01};
    canClient::sendFrame(0x22, 5, lightEnable.data());
    std::vector<unsigned char> lightOn{0x04, 0x00, 0x04, 0x00, 0x64};
    canClient::sendFrame(0x22, 5, lightOn.data());
}

void canClient::turnOffLight() 
{
    std::vector<unsigned char> lightOn{0x04, 0x00, 0x04, 0x00, 0x00};
    canClient::sendFrame(0x22, 5, lightOn.data());
}

void canClient::killRobot()
{
    canClient::sendFrame(0x00, 0, 0);
}

void canClient::allClear()
{
    canClient::sendFrame(0x00A, 0, 0);
}

std::vector<int> canClient::make_motor_request(std::vector<float>& thrusts, int motor_count, int max_power)
{
    #define MOTOR_ID 0x010
    /* Thrusts come out of PID as a float between -1 and 1; motors need int value from -100 to 100 */
    std::vector<int> convertedThrusts;
    for (float thrust : thrusts)
    {
        convertedThrusts.push_back(((int)(thrust * max_power)));
    }

    /* 
    * We have integer values that are 32 bits (4 bytes) but need values of one byte to send to motor
    * We can extract using an and mask and get last 8 bits which in hex is 0xFF. Char size is one byte
    * which is why we use an array of chars
    */          

    std::vector<unsigned char> byteThrusts;
    for (int thrust : convertedThrusts)
    {
        byteThrusts.push_back(thrust & 0xFF);
    }
    /* See exactly our 8 thrust values sent to motors */

////////////////////////////////////////// BUILD REQUEST //////////////////////////////////////////
    /* 
    * Our frame will send CAN request 
    * one byte for each value -100 to 100 
    */

    canClient::sendFrame(MOTOR_ID, motor_count, byteThrusts.data());
    return convertedThrusts;
}
