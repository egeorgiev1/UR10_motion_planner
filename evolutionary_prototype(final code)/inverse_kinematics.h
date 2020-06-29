#pragma once

#include <string>
#include <array>

#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainiksolver.hpp>

#include "util.h"

class InverseKinematics {
private:
    std::array<real_t, 3> _target_position {{0, 0, 0}};
	// std::array<real_t, 3> _target_orientation {{0, 0, 0}};
    KDL::Rotation         _target_orientation;
	std::array<real_t, 6> _current_angles {{0, 0, 0, 0, 0, 0}};
	std::string _last_error;

    KDL::Chain _joints_chain;
    
    KDL::ChainFkSolverPos* _kdl_fk_solver    { nullptr };
    KDL::ChainIkSolverVel* _velocity_solver  { nullptr };
    // TODO: Why LMA over Newtonian? How it handles singularities by going around
    // them??? Da vida dali taka behaveva i da vida da dam reference za tozi claim
    KDL::ChainIkSolverPos* _ik_solver { nullptr };

public:
    // Used for initializing ik_solver. If not done like this ik_solver would need to be a pointer.
    static KDL::Chain _joint_frames();

    InverseKinematics();

	void set_target_position(const std::array<real_t, 3>& position);
	
    void set_target_orientation(const std::array<real_t, 3>& orientation);

    void set_target_orientation(const std::array<real_t, 4>& orientation); // quaternion
	
    void set_current_angles(const std::array<real_t, 6>& current_angles);

    // If there's an error it will return -1, otherwise it will return 0
	intmax_t solve(std::array<real_t, 6>& output_angles);
	
    std::string get_last_error();
};
