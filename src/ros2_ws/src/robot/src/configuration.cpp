#include "configuration.hpp"
#include <fstream>
#include <stdio.h>

Configuration::Configuration(const std::string& filename)
{
    std::ifstream config_file(filename);
    if (!config_file.is_open()) {
        printf("Error: Could not open configuration file");
        this->json_string = "";
        return;
    }
    config_file >> this->json_string;
}