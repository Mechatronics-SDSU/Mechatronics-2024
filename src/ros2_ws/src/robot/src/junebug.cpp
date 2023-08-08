#include "junebug.hpp"

std::unique_ptr<Robot> Junebug::CreateJunebug(const Configuration& configuration)
{
    return std::make_unique<Junebug>(configuration);
}

Junebug::Junebug(const Configuration& configuration)
{
    this->name = "junebug";
}