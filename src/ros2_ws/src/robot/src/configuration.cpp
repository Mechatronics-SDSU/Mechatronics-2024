#include "configuration.hpp"
#include <fstream>
#include <iostream>

Configuration::Configuration(const std::string& filename)
{
    const std::string config_file_name = "config.json";
    std::ifstream config_file(config_file_name);
    if (!config_file.is_open()) {
        std::cerr << "Error: Could not open configuration file" << std::endl;
        return;
    }
    config_file >> this->json_string;
}


