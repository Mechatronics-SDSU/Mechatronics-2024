#include "component.hpp"
#include <cstring>

Component::Component(std::string name) : Node(name) 
{
    valid = false;
}

void Component::update()
{

}