#include "util.h"

real_t euclidean_distance(std::array<real_t, 3> p1, std::array<real_t, 3> p2) {

    return sqrt(
        (p1[0]-p2[0])*(p1[0]-p2[0]) +
        (p1[1]-p2[1])*(p1[1]-p2[1]) +
        (p1[2]-p2[2])*(p1[2]-p2[2])
    );
}

std::array<real_t, 3> lerp(std::array<real_t, 3> p1, std::array<real_t, 3> p2, real_t t) {
    return {{
        p1[0] + (p2[0] - p1[0])*t,
        p1[1] + (p2[1] - p1[1])*t,
        p1[2] + (p2[2] - p1[2])*t
    }};
}

std::array<real_t, 4> slerp(std::array<real_t, 4> q1_input, std::array<real_t, 4> q2_input, real_t t) {
    Eigen::Quaterniond q1(q1_input[0], q1_input[1], q1_input[2], q1_input[3]);
    Eigen::Quaterniond q2(q2_input[0], q2_input[1], q2_input[2], q2_input[3]);
    Eigen::Quaterniond result;

    result = q1.slerp(t, q2);

    return {{
        result.w(),
        result.x(),
        result.y(),
        result.z()
    }};
}