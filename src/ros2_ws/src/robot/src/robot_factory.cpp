#include "robot_factory.hpp"
#include "percy.hpp"
#include "junebug.hpp"
#include <unordered_map>
#include <cstring>
#include <functional>
#include <stdio.h>
#include <stdlib.h>

namespace 
{
    using RobotConstructor = std::function<std::shared_ptr<Robot>(const Configuration&)>;

    std::unordered_map<std::string, RobotType> createTypeMap()
    {
        return  std::unordered_map<std::string, RobotType> 
                {
                    {"percy", RobotType::Percy},
                    {"junebug", RobotType::Junebug}
                };
    }

    std::unordered_map<RobotType, RobotConstructor> createConstructorMap()
    {
        return  std::unordered_map<RobotType, RobotConstructor> 
                {
                    {RobotType::Percy, &Percy::CreatePercy},
                    {RobotType::Junebug, &Junebug::CreateJunebug}
                };
    }

    RobotType findType(std::string type_name, const std::unordered_map<std::string, RobotType>& robot_type_map)
    {
        auto it = robot_type_map.find(type_name);
        if (it != robot_type_map.end()) {
            return it->second;
        }
        fprintf(stderr, "\nInvalid robot type given in JSON file, reference file %s at line # %d - Sincerely, Zix ;)\n", __FILE__,__LINE__);
        exit(EXIT_FAILURE);
    }

    RobotConstructor findConstructor(RobotType type, const std::unordered_map<RobotType, RobotConstructor>& robot_constructor_map)
    {
        auto it = robot_constructor_map.find(type);
        if (it != robot_constructor_map.end()) {
            return it->second;
        }
        fprintf(stderr, "\nInvalid robot type given in JSON file, reference file %s at line # %d - Sincerely, Zix ;)\n", __FILE__,__LINE__);
        exit(EXIT_FAILURE);
    }

    RobotType getRobotType(const Configuration& config)
    {
        #define CONFIG_ROBOT_NAME_KEY "robot"
        nlohmann::json json_string = config.getJsonString();
        RobotType robot_type = findType(json_string[CONFIG_ROBOT_NAME_KEY], createTypeMap());
        return robot_type;
    }
}

std::shared_ptr<Robot> RobotFactory::createRobot(const Configuration& config)
{
    RobotType robot_type = getRobotType(config);
    RobotConstructor robot_constructor = findConstructor(robot_type, createConstructorMap());
    return robot_constructor(config);
}