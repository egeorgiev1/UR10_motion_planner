#pragma once

#include <array>

#include <Ogre.h>
#include <OgreApplicationContext.h>

#include "forward_kinematics.h"
#include "inverse_kinematics.h"
#include "robot_view.h"
#include "collision_view.h"
#include "collision_model.h"
#include "collision_controller.h"
#include "end_effector_view.h"
#include "input_manager.h"
#include "util.h"

class CartesianController : public OgreBites::InputListener {
private:
    // TODO: Rename delta to velocity and define exact units as m/s
    std::array<real_t, 3> _delta_position     {{0, 0, 0}};
	std::array<real_t, 3> _delta_orientation  {{0, 0, 0}};
	std::array<real_t, 3> _target_position    {{0, 0, 0}};
	std::array<real_t, 3> _target_orientation {{0, 0, 0}};

    ForwardKinematics*   _fk                    { nullptr };
    InverseKinematics*   _ik_solver             { nullptr };
    RobotView*           _robot_view            { nullptr };
    CollisionView*       _collision_view        { nullptr };
    CollisionModel*      _collision_model       { nullptr };
    CollisionController* _collision_controller  { nullptr };
    EndEffectorView*     _end_effector_view     { nullptr };
    InputManager*        _input_manager         { nullptr };

    void _handle_key_press_and_release(const OgreBites::KeyboardEvent& evt, bool is_pressed);

    bool _is_delta_zero();

public:
    CartesianController(
        ForwardKinematics* fk,
        InverseKinematics* ik_solver,
        RobotView* robot_view,
        CollisionView* collision_view,
        CollisionModel* collision_model,
        CollisionController* collision_controller,
        EndEffectorView* end_effector_view,
        InputManager* input_manager
    );

    std::array<real_t, 6> get_angles();
    
    // Toggle the different axes for position/orientation
	bool keyPressed(const OgreBites::KeyboardEvent& evt) override;
    
    // Toggle the different axes for position/orientation
	bool keyReleased(const OgreBites::KeyboardEvent& evt) override;

    // Increment the position/orientation axes using the deltas on every
    // frameRendered event. If the position or orientation is updated, because
    // the delta is different from zero then recalculate the IK. 
	void frameRendered (const Ogre::FrameEvent& evt) override;
};
