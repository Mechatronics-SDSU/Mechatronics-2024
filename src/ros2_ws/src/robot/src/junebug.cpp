#include "junebug.hpp"

std::unique_ptr<Robot> Junebug::CreateJunebug(const Configuration& configuration)
{
    return std::make_unique<Junebug>(configuration);
}

Junebug::Junebug(const Configuration& configuration)
{
    this->name = "junebug";
    this->motor_count = 2;
    this->thrust_mapper = Interface::matrix_t  {
                                                    {-1,  0,  0,  1,  0,  0},                   
                                                    { 1,  0,  0,  1,  0,  0},
                                               };
}