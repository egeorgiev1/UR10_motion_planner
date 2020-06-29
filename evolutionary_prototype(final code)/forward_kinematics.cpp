#include "forward_kinematics.h"

ForwardKinematics::ForwardKinematics (
    fcl::aligned_vector<fcl::Transform3d>* local_reference_frame,
    std::vector<Axis>& rotation_axis,
    std::vector<bool>& reverse_rotation_direction
) : // TODO: MUST check if all invariants are met and that the objects and pointers are valid!!!
    _local_reference_frame(local_reference_frame),
    _global_reference_frame(6, fcl::Transform3d::Identity()),
    _rotation_axis(rotation_axis),
    _invalidated(6, true), // HARDCODED 6 DEGREES OF FREEDOM???
    _joint_angles(6, 0.0 /* 0.0_r */), // HARDCODED 6 DEGREES OF FREEDOM???
    _reverse_rotation_direction(reverse_rotation_direction)
{}

std::array<real_t, 6> ForwardKinematics::get_angles() {
    std::array<real_t, 6> angles;

    for(intmax_t i = 0; i < 6; i++) {
        angles[i] = _joint_angles[i];
    }

    return angles;
}

void ForwardKinematics::set_angles(std::array<real_t, 6> joint_angles) {
    for(size_t i = 0; i < joint_angles.size(); i++) {
        set_angle(i, joint_angles[i]);
    }
}

void ForwardKinematics::set_angle(size_t joint_index, real_t angle) { // TODO: check invariants
    for(size_t i = joint_index; i < _invalidated.size(); i++) {
        _invalidated[i] = true;
    }

    _joint_angles[joint_index] = angle;
}

void ForwardKinematics::rotate(size_t joint_index, real_t angle) {
    if(_reverse_rotation_direction[joint_index]) {
        angle = (-angle);
    }

    set_angle(joint_index, _joint_angles[joint_index] + angle);
}

// No problem with returning Eigen objects: https://eigen.tuxfamily.org/dox/group__TopicPassingByValue.html
fcl::Transform3d ForwardKinematics::get_transform(size_t joint_index) {
    if(_invalidated[joint_index]) {
        // Find first index that's not invalidated
        ssize_t valid_transform_index;
        for(valid_transform_index = joint_index; valid_transform_index >= 0; valid_transform_index--) {
            if(!_invalidated[valid_transform_index]) {
                break;
            }
        }

        // Update transforms after the first valid index
        for(size_t i = valid_transform_index + 1; i <= joint_index; i++) {
            fcl::Vector3d axis_of_rotation;

            // HACK: Axes mapped to roll, pitch and yaw in Ogre3D: https://ogrecave.github.io/ogre/api/1.11/tiki-download_file_rotation_axis.png
            if(_rotation_axis[i] == Axis::X) {
                axis_of_rotation = -fcl::Vector3d::UnitZ(); // WHY DOES THIS NEED TO BE INVERSED???
            } else if(_rotation_axis[i] == Axis::Y) {
                axis_of_rotation = fcl::Vector3d::UnitY();
            } else if(_rotation_axis[i] == Axis::Z) {
                axis_of_rotation = fcl::Vector3d::UnitX();
            } else {
                // Error, but is it possible with enum class???
                throw "NO VALID AXIS SPECIFIED";
            }
            
            if(i == 0) {
                _global_reference_frame[i] = 
                    _local_reference_frame->at(i) *
                    fcl::AngleAxisd(_joint_angles[i], axis_of_rotation); // Why is the rotation last? Is the angle I'm using the angle for the joint after the current one???
            } else {
                _global_reference_frame[i] = 
                    _global_reference_frame[i - 1] *
                    _local_reference_frame->at(i) *
                    fcl::AngleAxisd(_joint_angles[i], axis_of_rotation);
            }

            _invalidated[i] = false;
        }
    }

    return _global_reference_frame[joint_index];
}

std::array<real_t, 3> ForwardKinematics::get_tcp_position() {
    fcl::Vector3d position {
        (get_transform(5) *
        // rpy="0.0 0.0 1.570796325" xyz="0.0 0.0922 0.0" - ORIGINAL
        // rpy="0.0 1.570796325 0.0" xyz="0.0 0.0 -0.0922" - TRANSFORMED
        fcl::Translation3d(0.0, 0.0, -0.0922) *
        fcl::AngleAxisd(0.0, fcl::Vector3d::UnitX()) *
        fcl::AngleAxisd(1.570796325, fcl::Vector3d::UnitY()) *
        fcl::AngleAxisd(0.0, fcl::Vector3d::UnitZ())
        ).translation()
    };

    return {{
        position.x(),
        position.y(),
        position.z()
    }};
}

std::array<real_t, 4> ForwardKinematics::get_tcp_orientation() {
    fcl::Quaterniond orientation {
        (get_transform(5) *
        // rpy="0.0 0.0 1.570796325" xyz="0.0 0.0922 0.0" - ORIGINAL
        // rpy="0.0 1.570796325 0.0" xyz="0.0 0.0 -0.0922" - TRANSFORMED
        fcl::Translation3d(0.0, 0.0, -0.0922) *
        fcl::AngleAxisd(0.0, fcl::Vector3d::UnitX()) *
        fcl::AngleAxisd(1.570796325, fcl::Vector3d::UnitY()) *
        fcl::AngleAxisd(0.0, fcl::Vector3d::UnitZ())
        ).rotation()
    };

    return {{
        orientation.w(),
        orientation.x(),
        orientation.y(),
        orientation.z()
    }};
}
