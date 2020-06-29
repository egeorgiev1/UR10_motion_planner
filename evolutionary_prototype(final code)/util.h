#pragma once

#include <array>
#include <cmath>
#include <eigen3/Eigen/Eigen>

typedef double real_t;
enum class Axis { X, Y, Z };

real_t euclidean_distance(std::array<real_t, 3> p1, std::array<real_t, 3> p2);

std::array<real_t, 3> lerp(std::array<real_t, 3> p1, std::array<real_t, 3> p2, real_t t);

std::array<real_t, 4> slerp(std::array<real_t, 4> p1, std::array<real_t, 4> p2, real_t t);

