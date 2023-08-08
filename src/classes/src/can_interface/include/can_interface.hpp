namespace canClient
{
    void sendFrame(int32_t can_id, int8_t can_dlc, unsigned char can_data[], Interface::ros_sendframe_client_t can_client);
    void setBotInSafeMode(Interface::ros_sendframe_client_t can_client);
    void turnOnLight(Interface::ros_sendframe_client_t can_client);
    void turnOffLight(Interface::ros_sendframe_client_t can_client);
}