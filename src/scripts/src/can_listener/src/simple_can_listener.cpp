#include <iostream>
#include <fstream>
#include <string>
#include <thread>
#include <chrono>
#include <cstring>
#include <unistd.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

const std::string canInterface = "can0"; 
const int targetSignalID = 0x04;       

void startTargetProgram() {
    const char* programPath = "/home/mechatronics/quantum/src/ros2_ws/install/robot_library/lib/robot_library/robot";
    
    pid_t pid = fork();
    if (pid == 0) {
        execl(programPath, programPath, nullptr);
        exit(EXIT_FAILURE);
    } else if (pid > 0) {
        int status;
        waitpid(pid, &status, 0);
    } else {
        std::cerr << "Fork failed!" << std::endl;
    }
}

int main() {
    int s;
    struct sockaddr_can addr;
    struct ifreq ifr;
    s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    strcpy(ifr.ifr_name, canInterface.c_str());
    ioctl(s, SIOCGIFINDEX, &ifr);
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    bind(s, (struct sockaddr *)&addr, sizeof(addr));
    struct can_frame frame;
    while (true) {
        int nbytes = read(s, &frame, sizeof(struct can_frame));
        if (nbytes < 0) {
            perror("Read error");
            break;
        }
        if (frame.can_id == targetSignalID) {
            printf("Button press signal received! Starting robot...\n");
            startTargetProgram();
        }
    }
    close(s);
    return 0;
}