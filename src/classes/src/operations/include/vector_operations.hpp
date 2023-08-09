#ifndef VECTOR_OPERATIONS_H
#define VECTOR_OPERATIONS_H
#define _USE_MATH_DEFINES

#include <iostream>
#include <vector>
#include <cmath>

// Print each element of vector
template <typename T>
void printVector(std::vector<T>& vector) 
{
    for (T element : vector)
    {
        std::cout << element << " ";
    }
    std::cout << std::endl;
}

// Decrement two vectors
template <typename T>
std::vector<T>& operator-=(std::vector<T> &lhs, const std::vector<T> &rhs) 
{
    if (lhs.size() != rhs.size())
        throw std::length_error("vectors must be same size to add");
    for (std::vector<double>::size_type i = 0; i < lhs.size(); ++i)
        lhs[i] -= rhs[i];
    return lhs;
}

// Subtract two vectors
template <typename T>
std::vector<T> operator- (std::vector<T> lhs, const std::vector<T> &rhs) 
{
    if (lhs.size() != rhs.size())
        throw std::length_error("vectors must be same size to add");
    return lhs -= rhs;
}

// Increment two vectors
template <typename T>
std::vector<T>& operator+=(std::vector<T> &lhs, const std::vector<T> &rhs) 
{
    if (lhs.size() != rhs.size())
        throw std::length_error("vectors must be same size to add");
    for (std::vector<double>::size_type i = 0; i < lhs.size(); ++i)
        lhs[i] += rhs[i];
    return lhs;
}

// Add two vectors
template <typename T>
std::vector<T> operator+ (std::vector<T> lhs, const std::vector<T> &rhs) 
{
    if (lhs.size() != rhs.size())
        throw std::length_error("vectors must be same size to add");
    return lhs += rhs;
}

// Subtract number from each element of vector
template <typename T>
std::vector<T> operator-(std::vector<T> lhs, const T &rhs) 
{
    for (std::vector<double>::size_type i = 0; i < lhs.size(); ++i)
        lhs[i] -= rhs;
    return lhs;
}

// Add number to each element of vector
template <typename T>
std::vector<T> operator+(std::vector<T> lhs, const T &rhs) 
{
    for (std::vector<double>::size_type i = 0; i < lhs.size(); ++i)
        lhs[i] += rhs;
    return lhs;
}


// See if number is less than each element of vector
template <typename T>
bool operator<(std::vector<T> lhs, const T &rhs) 
{
    bool lessThan = true;
    for (std::vector<double>::size_type i = 0; i < lhs.size(); ++i)
        if (lhs[i] > rhs)
        {
            lessThan = false;
        }
    return lessThan;
}

// See if number is greater than each element of vector
template <typename T>
bool operator>(std::vector<T> lhs, const T &rhs) 
{
    bool greaterThan = true;
    for (std::vector<double>::size_type i = 0; i < lhs.size(); ++i)
        if (lhs[i] < rhs)
        {
            greaterThan = false;
        }
    return greaterThan;
}

// Multiply number to each element of vector
template <typename T>
std::vector<T> operator*(std::vector<T> lhs, const T &rhs) 
{
    for (std::vector<double>::size_type i = 0; i < lhs.size(); ++i)
        lhs[i] *= rhs;
    return lhs;
}

// Add two vectors
template <typename T>
bool operator== (std::vector<T> lhs, const std::vector<T> &rhs) 
{
    bool equal = true;
    for (std::vector<double>::size_type i = 0; i < lhs.size(); ++i)
    {
        if (lhs[i] != rhs[i]) {equal = false;}
    }
    return equal;
}

// Divide each element of vector by number
template <typename T>
std::vector<T> operator/(std::vector<T> lhs, const T &rhs) 
{
    for (std::vector<double>::size_type i = 0; i < lhs.size(); ++i)
        lhs[i] /= rhs;
    return lhs;
}

// matrix multiplication with two-dimensional and one-dimensional array
template<typename T>
std::vector<T> operator*(std::vector<std::vector<T>> lhs, const std::vector<T> &rhs)
{
    std::vector<T> matrix;

    for (std::vector<T> row : lhs)
    {
        T newElement = 0;
        for (std::vector<double>::size_type i = 0; i < rhs.size(); i++)
        {
            newElement += row[i] * rhs[i];
        }     
    matrix.push_back(newElement);
    }

    return matrix;
}

template<typename T>
std::vector<T> abs(std::vector<T>& vect)
{
    for (T& value : vect)
    {
        value = abs(value);
    }
    return vect;
}

template<typename T>
T sum(std::vector<T>& vect)
{
    T sum = 0;
    for (T& value : vect)
    {
        sum += value;
    }
    return sum;
}

#endif /* VECTOR_OPERATIONS_H */