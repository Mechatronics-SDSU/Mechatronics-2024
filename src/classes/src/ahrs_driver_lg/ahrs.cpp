#include <iostream>
#include <string>
#include <vector>
#include <cstring>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <map>

class SpartonAHRSDataPackets {
public:
    SpartonAHRSDataPackets(const std::string& com_port = "/dev/ttyUSB0") : ahrs_serial(openSerialPort(com_port)) {
        location_array = {
            {"raw_magnetics", {0x01, 9}}, {"true_heading", {0x02, 5}},
            {"magnetic_heading", {0x09, 5}}, {"magnetic_variation", {0x83, 5}},
            {"auto_magnetic_variation", {0x0F, 5}}, {"latitude", {0x8B, 5}},
            {"longitude", {0x8C, 5}}, {"altitude", {0x8D, 5}}, {"day", {0x8E, 5}},
            {"magnetic_vector", {0x04, 11}}, {"raw_acceleration", {0x05, 9}},
            {"pitch_roll", {0x06, 7}}, {"accleration_vector", {0x07, 11}},
            {"temperature", {0x11, 5}}, {"baud_rate", {0x57, 4}},
            {"mounting_config", {0x4A, 4}}
        };
    }

    std::vector<uint8_t> get_raw_magnetics() {
        sendRequest({0xA4, 0x01, 0xA0});
        return unpack("raw_magnetics");
    }

    double get_true_heading() {
        sendRequest({0xA4, 0x02, 0xA0});
        std::vector<uint8_t> true_heading_data = unpack("true_heading");
        if (true_heading_data.size() == 2) {
            int16_t raw_value = (true_heading_data[0] << 8) | true_heading_data[1];
            return raw_value * (360.0 / 4096.0);
        }
        return -1.0; // Indicate failure
    }

    std::vector<double> get_pitch_roll() {
        sendRequest({0xA4, 0x06, 0xA0});
        std::vector<uint8_t> pitch_roll_data = unpack("pitch_roll");
        if (pitch_roll_data.size() == 4) {
            double pitch = (static_cast<int16_t>((pitch_roll_data[0] << 8) | pitch_roll_data[1])) * (90.0 / 4096.0);
            double roll = (static_cast<int16_t>((pitch_roll_data[2] << 8) | pitch_roll_data[3])) * (180.0 / 4096.0);
            return {pitch, roll};
        }
        return {0, 0}; // Indicate failure
    }

private:
    int ahrs_serial;
    const uint8_t success_header_byte = 0xA4;
    const uint8_t error_header_byte = 0xAE;
    const uint8_t termination_byte = 0xA0;
    std::map<std::string, std::pair<uint8_t, int>> location_array;

    int openSerialPort(const std::string& com_port) {
        int serial_port = open(com_port.c_str(), O_RDWR);
        if (serial_port == -1) {
            perror("Error opening serial port");
            exit(1);
        }
        termios tty;
        memset(&tty, 0, sizeof(tty));
        if (tcgetattr(serial_port, &tty) != 0) {
            perror("Error from tcgetattr");
            exit(1);
        }
        tty.c_cflag &= ~PARENB;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag |= CS8;
        tty.c_cflag &= ~CRTSCTS;
        tty.c_cflag |= CREAD | CLOCAL;
        tty.c_lflag &= ~ICANON;
        tty.c_lflag &= ~ECHO;
        tty.c_lflag &= ~ECHOE;
        tty.c_lflag &= ~ECHONL;
        tty.c_lflag &= ~ISIG;
        tty.c_iflag &= ~(IXON | IXOFF | IXANY);
        tty.c_iflag &= ~(ICRNL | INLCR);
        tty.c_oflag &= ~OPOST;
        tty.c_cc[VMIN] = 1;
        tty.c_cc[VTIME] = 0;
        cfsetospeed(&tty, B115200);
        cfsetispeed(&tty, B115200);
        if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
            perror("Error from tcsetattr");
            exit(1);
        }
        return serial_port;
    }

    void sendRequest(const std::vector<uint8_t>& request) {
        write(ahrs_serial, request.data(), request.size());
    }

    std::vector<uint8_t> unpack(const std::string& data_type) {
        std::vector<uint8_t> ahrs_data_in;
        usleep(1000);
        while (true) {
            uint8_t header_byte;
            if (read(ahrs_serial, &header_byte, 1) <= 0) {
                return {}; // No data received
            }
            if (header_byte == success_header_byte) {
                uint8_t type_byte;
                if (read(ahrs_serial, &type_byte, 1) <= 0) {
                    return {}; // No data received
                }
                if (type_byte == location_array[data_type].first) {
                    for (int idx = 0; idx < location_array[data_type].second - 2; ++idx) {
                        uint8_t data_byte;
                        if (read(ahrs_serial, &data_byte, 1) <= 0) {
                            return {}; // No data received
                        }
                        ahrs_data_in.push_back(data_byte);
                    }
                    uint8_t termination;
                    if (read(ahrs_serial, &termination, 1) <= 0) {
                        return {}; // No data received
                    }
                    if (termination != termination_byte) {
                        return {}; // Incorrect termination
                    }
                    return ahrs_data_in;
                }
            } else if (header_byte == error_header_byte) {
                return {}; // Error header byte
            }
        }
    }
};

int main(int argc, char* argv[]) {
    if (argc > 1) {
        std::string ahrs_dev_port = argv[1];
        SpartonAHRSDataPackets ahrs(ahrs_dev_port);
        std::cout << "Post connect" << std::endl;
        while (true) {
            double yaw = ahrs.get_true_heading();
            std::vector<double> pitch_roll = ahrs.get_pitch_roll();
            if (yaw >= 0.0) {
                std::cout << "Pitch: " << pitch_roll[0] << ", Roll: " << pitch_roll[1] << ", Yaw: " << yaw << std::endl;
            }
            usleep(10000);
        }
    } else {
        std::cerr << "Error: not enough arguments. (Was the AHRS dev port passed?)" << std::endl;
        return 1;
    }
    return 0;
}
