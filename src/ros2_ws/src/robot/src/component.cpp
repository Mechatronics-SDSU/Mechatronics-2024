#include "component.hpp"

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
    this->validated = true;
}