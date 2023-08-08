#include "percy.hpp"

std::unique_ptr<Robot> Percy::CreatePercy(const Configuration& configuration)
{
    return std::make_unique<Percy>(configuration);
}

Percy::Percy(const Configuration& configuration)
{
    this->name_ = "percy";
}