#include "can_interface.hpp"

void canClient::sendFrame(int32_t can_id, int8_t can_dlc, unsigned char can_data[], Interface::ros_sendframe_client_t can_client)
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
}

void canClient::setBotInSafeMode(Interface::ros_sendframe_client_t can_client)
{
    std::vector<unsigned char> safeModeFrame{0,0,0,0,0x04};
    sendFrame(0x022, 5, safeModeFrame.data(), can_client);
}

void canClient::turnOnLight(Interface::ros_sendframe_client_t can_client) 
{
    std::vector<unsigned char> lightEnable{0x04, 0x00, 0x00, 0x00, 0x01};
    canClient::sendFrame(0x22, 5, lightEnable.data(), can_client);
    std::vector<unsigned char> lightOn{0x04, 0x00, 0x04, 0x00, 0x64};
    canClient::sendFrame(0x22, 5, lightOn.data(), can_client);
}

void canClient::turnOffLight(Interface::ros_sendframe_client_t can_client) 
{
    std::vector<unsigned char> lightOn{0x04, 0x00, 0x04, 0x00, 0x00};
    canClient::sendFrame(0x22, 5, lightOn.data(), can_client);
}