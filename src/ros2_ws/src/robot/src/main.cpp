#include "robot.hpp"

int main(int argc, char** argv)
{
    std::unique_ptr<Robot> robot = std::make_unique<Robot>();
    return EXIT_SUCCESS;
}