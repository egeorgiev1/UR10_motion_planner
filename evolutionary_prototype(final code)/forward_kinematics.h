#pragma once

#include <vector>
#include <array>

#include <fcl/fcl.h>

#include "util.h"
#include "observable.h"

class ForwardKinematics {
private:
    fcl::aligned_vector<fcl::Transform3d>* _local_reference_frame;
    std::vector<Axis> _rotation_axis;
    std::vector<bool> _reverse_rotation_direction;

    fcl::aligned_vector<fcl::Transform3d> _global_reference_frame; // TODO: remove fcl dependency here??? Use Eigen directly???
    std::vector<bool> _invalidated; // So that _global_reference_frame is lazily evaluated without unnecessary matrix multiplications
    std::vector<real_t> _joint_angles;
public:
    ForwardKinematics(
        fcl::aligned_vector<fcl::Transform3d>* local_reference_frame,
        std::vector<Axis>& rotation_axis,
        std::vector<bool>& reverse_rotation_direction
    );

    std::array<real_t, 6> get_angles();

    void set_angles(std::array<real_t, 6> joint_angles);

    void set_angle(size_t joint_index, real_t angle);

    void rotate(size_t joint_index, real_t angle);

    // No problem with returning Eigen objects: https://eigen.tuxfamily.org/dox/group__TopicPassingByValue.html
    fcl::Transform3d get_transform(size_t joint_index);

    std::array<real_t, 3> get_tcp_position();

    std::array<real_t, 4> get_tcp_orientation(); // quaternion
};
