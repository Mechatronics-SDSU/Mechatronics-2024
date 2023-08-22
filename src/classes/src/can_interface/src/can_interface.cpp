#include "can_interface.hpp"

CanInterface::CanClient::CanClient()
{
    this->node = rclcpp::Node::make_shared("can_client");
    this->can_client = node->create_client<scion_types::srv::SendFrame>("send_can_raw");
}

void CanInterface::CanClient::sendFrame(int32_t can_id, int8_t can_dlc, unsigned char can_data[])
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
    RCLCPP_INFO(node->get_logger(), "[CanSendService::client] Client Sent Message With Id %03x", can_id);
}

void CanInterface::CanClient::sendNothing()
{
    std::vector<unsigned char> nothing{0x00};
    sendFrame(0x010, 0, nothing.data());
}

void CanInterface::CanClient::setBotInSafeMode()
{
    std::vector<unsigned char> safeModeFrame{0,0,0,0,0x04};
    sendFrame(0x022, 5, safeModeFrame.data());
}

void CanInterface::CanClient::turnOnLight() 
{
    std::vector<unsigned char> lightEnable{0x04, 0x00, 0x00, 0x00, 0x01};
    CanInterface::CanClient::sendFrame(0x22, lightEnable.size(), lightEnable.data());
    std::vector<unsigned char> lightOn{0x04, 0x00, 0x04, 0x00, 0x64};
    CanInterface::CanClient::sendFrame(0x22, lightEnable.size(), lightOn.data());
}

void CanInterface::CanClient::turnOffLight() 
{
    std::vector<unsigned char> lightOn{0x04, 0x00, 0x04, 0x00, 0x00};
    CanInterface::CanClient::sendFrame(0x22, 5, lightOn.data());
}

void CanInterface::CanClient::killRobot()
{
    CanInterface::CanClient::sendFrame(0x00, 0, 0);
}

void CanInterface::CanClient::allClear()
{
    CanInterface::CanClient::sendFrame(0x00A, 0, 0);
}

std::vector<int> CanInterface::CanClient::make_motor_request(const std::vector<float>& thrusts, int motor_count, int max_power)
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

    CanInterface::CanClient::sendFrame(MOTOR_ID, motor_count, byteThrusts.data());
    return convertedThrusts;
}
