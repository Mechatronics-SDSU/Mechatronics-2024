#pragma once
#include <nlohmann/json.hpp>
#include <cstring>

class Configuration
{
    public:
        Configuration(const std::string& filename);
    private:
        nlohmann::json json_string;
};