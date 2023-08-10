#include "math_operations.hpp"

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