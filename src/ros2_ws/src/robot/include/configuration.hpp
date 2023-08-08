#pragma once
#include <nlohmann/json.hpp>
#include <cstring>

class Configuration
{
    public:
        Configuration(const std::string& filename);
        nlohmann::json getJsonString() {return this->json_string;}
    private:
        nlohmann::json json_string;
};