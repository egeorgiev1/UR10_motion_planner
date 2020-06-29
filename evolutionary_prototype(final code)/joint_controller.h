#pragma once

#include <Ogre.h>
#include <OgreApplicationContext.h>

#include "robot_view.h"
#include "collision_view.h"
#include "forward_kinematics.h"
#include "end_effector_view.h"
#include "input_manager.h"
#include "util.h"

class JointController : public OgreBites::InputListener {
private:
    RobotView*         _robot_view        { nullptr };
    CollisionView*     _collision_view    { nullptr };
    EndEffectorView*   _end_effector_view { nullptr };
    ForwardKinematics* _fk                { nullptr };
    InputManager*      _input_manager     { nullptr };

    real_t _rotation_delta { 0.0 };
    uintmax_t _selected_rotation_frame { 0 };
public:
    JointController(
        RobotView* robot_view,
        CollisionView* collision_view,
        ForwardKinematics* fk,
        InputManager* input_manager,
        EndEffectorView* end_effector_view
    );

	bool keyPressed(const OgreBites::KeyboardEvent& evt) override;
    
	bool keyReleased(const OgreBites::KeyboardEvent& evt) override;

	void frameRendered(const Ogre::FrameEvent& evt) override;
};