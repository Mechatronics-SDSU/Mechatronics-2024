#ifndef FINITE_IMPULSE_FILTER_H
#define FINITE_IMPULSE_FILTER_H

#include <vector>
#include <deque>
#include <array>
#include <string>

class FiniteImpulseFilter
{
    public:
        FiniteImpulseFilter(size_t targets, std::string& coeff_file);
        float smooth(std::deque<float>& static_deque, float in_x);
        void print_coefficients();
        std::vector<std::deque<float>> data_streams;         // Dynamic amount of static (by class member) data streams
    private:
        std::vector<float> coefficients;
        void read_coeffs_from_file(std::vector<float>&, const std::string& filename);
};

#endif