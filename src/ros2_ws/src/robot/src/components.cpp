#include <stdio.h>
#include <stdlib.h>
#include "components.hpp"
#include "gui_listener_node.hpp"
#include "controller_node.hpp"
#include "unified_can_driver.hpp"
#include "pid_node.hpp"
#include "sub_state_listener_node.hpp"

namespace
{
    #define NODES_MAP_KEY "nodes"
    typedef std::shared_ptr<std::vector<std::shared_ptr<Component>>> component_vector_ptr_t;
}

namespace
{
    std::shared_ptr<Component> createComponent(const Robot& robot, std::string component_name)
    {
        if (component_name == "gui_listener")           {return std::make_shared<GUI_Listener>();}
        if (component_name == "unified_can_driver")     {return std::make_shared<UnifiedCanDriver>();}
        if (component_name == "sub_state_listener")     {return std::make_shared<SubStateListenerNode>();}
        if (component_name == "controller_node")        {return std::make_shared<Controller>(robot);}
        if (component_name == "pid_node")               {return std::make_shared<PidNode>(robot);}
        fprintf(stderr, "\nInvalid node given in JSON file, exception thrown at %s at line # %d - Sincerely, Zix ;)\n", __FILE__,__LINE__);
        exit(EXIT_FAILURE);
    }
}

namespace
{
    component_vector_ptr_t allocateComponentsVector()
    {
        return std::make_shared<std::vector<std::shared_ptr<Component>>>();
    }

    component_vector_ptr_t addComponent(const std::shared_ptr<Component> component, component_vector_ptr_t component_vector)
    {
        component_vector->push_back(component); 
        return component_vector;
    }

    component_vector_ptr_t populateComponentsFromStringList(const Robot& robot, const std::vector<std::string>& nodes_to_enable, component_vector_ptr_t component_vector)
    {
        for (std::string component_name : nodes_to_enable)
        {
            addComponent(createComponent(robot, component_name), component_vector);
        }
        return component_vector;
    }

    const std::vector<std::string> getNodesToEnable(Robot& robot)
    {
        return robot.getConfiguration().getJsonString()[NODES_MAP_KEY];
    }
}

component_vector_ptr_t Components::CreateComponentVector(Robot& robot)
{
    return populateComponentsFromStringList(robot, getNodesToEnable(robot), allocateComponentsVector());
}
