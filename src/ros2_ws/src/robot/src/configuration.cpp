#include "configuration.hpp"
#include <fstream>
#include <iostream>

Configuration::Configuration(const std::string& filename)
{
    std::ifstream config_file(filename);
    if (!config_file.is_open()) {
        std::cerr << "Error: Could not open configuration file" << std::endl;
        return;
    }
    config_file >> this->json_string;
}


