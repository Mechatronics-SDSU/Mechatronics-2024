#include <vector>
#include <memory>
#include <deque>

namespace mathOperations
{
    std::vector<float> normalizeCtrlVals(std::vector<float>& ctrl_vals);
    bool areEqual(float float1, float float2, float epsilon);
    bool areEqual(std::vector<float>& current_state, std::vector<float>& desired_state);
    bool equalToZero(std::vector<int> thrustVect);
    int calculateTotalSlew(std::deque<std::vector<int>>& slew_buffer);
}