#include "percy.hpp"

std::unique_ptr<Robot> Percy::CreatePercy(const Configuration& configuration)
{
    return std::make_unique<Percy>(configuration);
}

Percy::Percy(const Configuration& configuration) : Robot()
{
    this->name = "percy";
    this->motor_count = 8;
    this->thrust_mapper = Interface::matrix_t   {
                                                    { 0, -1, -1,  0,  0,  1},
                                                    {.9,  0,  0, .9, .9,  0},
                                                    { 0,  1, -1,  0,  0,  1},
                                                    {.9,  0,  0, .9,-.9,  0},
                                                    { 0,  1,  1,  0,  0,  1},
                                                    { 1,  0,  0, -1, -1,  0},
                                                    { 0, -1,  1,  0,  0,  1},
                                                    { 1,  0,  0, -1,  1,  0}
                                                };
}