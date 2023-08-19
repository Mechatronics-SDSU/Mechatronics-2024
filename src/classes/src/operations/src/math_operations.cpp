#include "math_operations.hpp"
#include "vector_operations.hpp"
#include <cmath>

std::vector<float> mathOperations::normalizeCtrlVals(std::vector<float>& ctrl_vals)
{
    std::vector<float> normalized{0,0,0,0,0,0};
    
    float vectorTotal = abs(ctrl_vals[0]) + abs(ctrl_vals[3]) + abs(ctrl_vals[4]); // yaw, x, y
    float nonVectorTotal = abs(ctrl_vals[1]) + abs(ctrl_vals[2]) + abs(ctrl_vals[5]); // roll, pitch, z

    normalized[0] = ctrl_vals[0];
    normalized[1] = ctrl_vals[1];
    normalized[2] = ctrl_vals[2];
    normalized[3] = ctrl_vals[3];
    normalized[4] = ctrl_vals[4];
    normalized[5] = ctrl_vals[5];

    if (vectorTotal > 1)
    {
        normalized[0] = ctrl_vals[0] / vectorTotal;
        normalized[3] = ctrl_vals[3] / vectorTotal;
        normalized[4] = ctrl_vals[4] / vectorTotal;
    }
    if (nonVectorTotal > 1)
    {
        normalized[1] = ctrl_vals[1] / nonVectorTotal;
        normalized[2] = ctrl_vals[2] / nonVectorTotal;
        normalized[5] = ctrl_vals[5] / nonVectorTotal;
    }

    if (vectorTotal == 0)
    {
        normalized[0] = 0;
        normalized[3] = 0;
        normalized[4] = 0;
    }

    if (nonVectorTotal == 0)
    {
        normalized[1] = 0;
        normalized[2] = 0;
        normalized[5] = 0;
    }

    return normalized;
}

bool mathOperations::areEqual(float float1, float float2, float epsilon)
{
    return (fabs(float1 - float2) < epsilon);
}

bool mathOperations::areEqual(std::vector<float>& current_state, std::vector<float>& desired_state)
{
    #define ORIENTATION_TOLERANCE 4.0f
    #define POSITION_TOLERANCE 0.06f

    bool equal = true;
    for (std::vector<float>::size_type i = 0; i < 3; i++)
    {
        if (!areEqual(current_state[i], desired_state[i], ORIENTATION_TOLERANCE)) //.05*current_state[i])
        {
            equal = false;
        }
    }
    for (std::vector<float>::size_type j = 3; j < 6; j++)
    {
        if (!areEqual(current_state[j], desired_state[j], POSITION_TOLERANCE)) //.05*current_state[i])
        {
            equal = false;
        }
    }
    return equal;
}

bool mathOperations::equalToZero(std::vector<int> thrustVect)
{
    bool equal = true;
    for (int thrust : thrustVect)
    {
        if (thrust > 1) {equal = false;}
    }
    return equal;
}

int mathOperations::calculateTotalSlew(std::deque<std::vector<int>>& slew_buffer)
{
    int slew_rate = 0;
    for (size_t i = 0; i < slew_buffer.size() - 1; i++)
    {   
        std::vector<int> difference = abs(slew_buffer[i]) - abs(slew_buffer[i+1]);
        slew_rate += abs(sum(difference));
    }
    return slew_rate;
}