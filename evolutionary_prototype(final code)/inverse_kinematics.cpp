#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr.hpp>

#include "inverse_kinematics.h"

#include <iostream>

using namespace std;
using namespace KDL;

// Used for initializing ik_solver. If not done like this ik_solver would need to be a pointer.
KDL::Chain InverseKinematics::_joint_frames() {
    KDL::Chain _joint_frames;

    // Setup joint frames of the UR10 using DH parameters
    _joint_frames.addSegment(Segment(Joint(Joint::RotZ), Frame::DH(0,        M_PI_2,  0.1273,   0)));
    _joint_frames.addSegment(Segment(Joint(Joint::RotZ), Frame::DH(-0.612,   0,       0,        0)));
    _joint_frames.addSegment(Segment(Joint(Joint::RotZ), Frame::DH(-0.5723,  0,       0,        0)));
    _joint_frames.addSegment(Segment(Joint(Joint::RotZ), Frame::DH(0,        M_PI_2,  0.163941, 0)));
    _joint_frames.addSegment(Segment(Joint(Joint::RotZ), Frame::DH(0,       -M_PI_2,  0.1157,   0)));
    _joint_frames.addSegment(Segment(Joint(Joint::RotZ), Frame::DH(0,        0,       0.0922,   0)));

    return _joint_frames;
}

InverseKinematics::InverseKinematics()
:
    _joints_chain{_joint_frames()}
{
    // _kdl_fk_solver = new ChainFkSolverPos_recursive { _joints_chain };
    // _velocity_solver = new ChainIkSolverVel_pinv {
    //     _joints_chain,
    //     0.00001,
    //     150
    // };

    // _ik_solver = new ChainIkSolverPos_NR {
    //     _joints_chain,
    //     *_kdl_fk_solver,
    //     *_velocity_solver,
    //     1000,
    //     1e-3
    // };

    // PROBVANE SAS DRUG SOLVER, kadeto moga bez problem da increasena samia precision???
    _ik_solver = new ChainIkSolverPos_LMA{
        _joints_chain,
        1e-5,
        500, // old: 50000
        1e-15
    };
}

void InverseKinematics::set_target_position(const std::array<real_t, 3>& position) {
    _target_position = position;
}

void InverseKinematics::set_target_orientation(const std::array<real_t, 3>& orientation) { // In RPY format
    _target_orientation = KDL::Rotation::RPY(
        orientation[0],
        orientation[1],
        orientation[2]
    );

    // DEBUG
    real_t w { 0.0 }, x { 0.0 }, y { 0.0 }, z { 0.0 };
    _target_orientation.GetQuaternion(w, x, y, z);
    cout << "IK Quaternion orientation: " << endl;
    cout << "W: " << w << endl;
    cout << "X: " << x << endl;
    cout << "Y: " << y << endl;
    cout << "Z: " << z << endl;
}

void InverseKinematics::set_target_orientation(const std::array<real_t, 4>& orientation) { // Quaternion
    _target_orientation = KDL::Rotation::Quaternion(
        orientation[0],
        orientation[1],
        orientation[2],
        orientation[3]
    );
}

void InverseKinematics::set_current_angles(const std::array<real_t, 6>& current_angles) {
    _current_angles = current_angles;
}

// If there's an error it will return -1, otherwise it will return 0
intmax_t InverseKinematics::solve(std::array<real_t, 6>& output_angles) {
    // Create input and output arrays of angles
    JntArray initial_angles(_current_angles.size());
    JntArray resulting_angles(_current_angles.size());
    
    // Initialize input array of angles
    for(size_t i = 0; i < _current_angles.size(); i++) {
        initial_angles(i) = _current_angles[i];
    }
    
    // Create destination frame
    Frame destination_frame = Frame(
        _target_orientation,
        KDL::Vector(_target_position[0], _target_position[1], _target_position[2])
    );

    // Run the solver
    int error_code = _ik_solver->CartToJnt(initial_angles, destination_frame, resulting_angles);

    // Set output angles
    for(size_t i = 0; i < _current_angles.size(); i++) {
        output_angles[i] = resulting_angles(i);
    }

    // Set error string
    if(error_code == KDL::SolverI::E_NOERROR) {
        _last_error = ""s;
    } else {
        _last_error = _ik_solver->strError(error_code);
    }

    // Return 0 if no error, otherwise return -1
    if(error_code == KDL::SolverI::E_NOERROR) {
        return 0;
    } else {
        return -1;
    }
}

std::string InverseKinematics::get_last_error() {
    return _last_error;
}
