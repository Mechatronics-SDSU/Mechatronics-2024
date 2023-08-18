#pragma once
#include <nlohmann/json.hpp>
#include <cstring>

class Configuration
{
    public:
        Configuration(const std::string& filename);
        nlohmann::json getJsonString() const {return this->json_string;}
    protected:
        nlohmann::json json_string;
};
