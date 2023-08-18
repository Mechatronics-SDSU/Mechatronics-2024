#include "components.hpp"
#include "gui_listener_node.hpp"
#include "controller_node.hpp"

namespace
{
    #define NODES_MAP_KEY "nodes"
    typedef std::shared_ptr<std::vector<std::shared_ptr<Component>>> component_vector_ptr_t;
}

namespace
{
    std::shared_ptr<Component> createComponent(const Robot& robot, std::string component_name)
    {
        if (component_name == "gui_listener")       {return std::make_shared<GUI_Listener>();}
        if (component_name == "controller_node")    {return std::make_shared<Controller>(robot);}
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
}

Components::Components(Robot& robot) : robot{robot} {}

component_vector_ptr_t Components::CreateComponentVector()
{
    return populateComponentsFromStringList(this->robot, this->robot.getConfiguration().getJsonString()[NODES_MAP_KEY], allocateComponentsVector());
}



