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
#include "motion_planner.h"
#include "util.h"

class TrajectoryController : public OgreBites::InputListener {
private:
    // TODO: Rename delta to velocity and define exact units as m/s
    real_t _current_step { 0.0 };
    real_t _step_speed { 0.0 };
    real_t _step_delta { 0 }; // -step_speed for backwards, 0 for no movement and step_speed for moving forwards
    trajectory_t _trajectory;

    ForwardKinematics*   _fk                    { nullptr };
    RobotView*           _robot_view            { nullptr };
    CollisionView*       _collision_view        { nullptr };
    CollisionModel*      _collision_model       { nullptr };
    CollisionController* _collision_controller  { nullptr };
    EndEffectorView*     _end_effector_view     { nullptr };
    InputManager*        _input_manager         { nullptr };

    void _handle_key_press_and_release(const OgreBites::KeyboardEvent& evt, bool is_pressed);

public:
    TrajectoryController(
        ForwardKinematics* fk,
        RobotView* robot_view,
        CollisionView* collision_view,
        CollisionModel* collision_model,
        CollisionController* collision_controller,
        EndEffectorView* end_effector_view,
        InputManager* input_manager,
        real_t step_speed
    );

    void set_trajectory(const trajectory_t& trajectory);
    
    // Toggle directional movement along the trajectory
	bool keyPressed(const OgreBites::KeyboardEvent& evt) override;
    
    // Toggle directional movement along the trajectory
	bool keyReleased(const OgreBites::KeyboardEvent& evt) override;

    // Move along the trajectory one step per frame
	void frameRendered (const Ogre::FrameEvent& evt) override;
};
