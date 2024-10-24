#pragma once

#include <vector>
#include <array>
#include <iostream>

// Matrix Multiplciation
std::vector<double> matMult(const std::array<std::array<double, 3>, 3> arr, const std::vector<double> pqr_input);

std::vector<double> matMult(const std::vector<double> vec, const double scalar);

std::array<std::array<double, 3>, 3> matMult(const std::array<std::array<double, 3>, 3> arr1, const std::array<std::array<double, 3>, 3> arr2);

std::array<std::array<double, 3>, 3> matMult(const std::array<std::array<double, 3>, 3> arr, const double scalar);

std::array<std::array<double, 3>, 3> matAdd(const std::array<std::array<double, 3>, 3> arr1, const std::array<std::array<double, 3>, 3> arr2);

std::vector<double> matAdd(const std::vector<double> vec1, const std::vector<double> vec2);

std::array<std::array<double, 3>, 3> getCdot(std::array<std::array<double, 9>, 2> input);

std::vector<double> getXdot(std::array<std::array<double, 9>, 2> input);