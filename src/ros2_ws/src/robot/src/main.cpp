#include "robot.hpp"
#include "configuration.hpp"
#define CONFIG_FILE "config.json"

int main(int argc, char** argv)
{
    std::unique_ptr<Configuration> config = std::make_unique<Configuration>(CONFIG_FILE);
    std::unique_ptr<Robot> robot = std::make_unique<Robot>(*config);
    return EXIT_SUCCESS;
}