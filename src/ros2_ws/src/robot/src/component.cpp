#include "component.hpp"
#include <cstring>

Component::Component(std::string name) : Node(name)
{

}

void Component::enable()
{
    this->enabled = true;
}

void Component::disable()
{
    this->enabled = false;
}

void Component::validate()
{
    this->valid = true;
}