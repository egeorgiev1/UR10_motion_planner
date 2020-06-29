#include "joint_controller.h"

JointController::JointController(
    RobotView* robot_view,
    CollisionView* collision_view,
    ForwardKinematics* fk,
    InputManager* input_manager,
    EndEffectorView* end_effector_view
) :
    _robot_view(robot_view),
    _collision_view(collision_view),
    _fk(fk),
    _input_manager(input_manager),
    _end_effector_view(end_effector_view)
{
    _input_manager->add_listener(this);
}

bool JointController::keyPressed(const OgreBites::KeyboardEvent& evt) {
    if (evt.keysym.sym >= '1' && evt.keysym.sym <= '6' && (bool)evt.repeat == false) {
        _selected_rotation_frame = evt.keysym.sym - '1';
    }
    if(evt.keysym.sym == '[') {
        _rotation_delta = -0.5;
    }

    if(evt.keysym.sym == ']') {
        _rotation_delta = 0.5;
    }
}

bool JointController::keyReleased(const OgreBites::KeyboardEvent& evt) {
    if(evt.keysym.sym == '[' || evt.keysym.sym == ']') {
        _rotation_delta = 0.0;
    }
}

void JointController::frameRendered(const Ogre::FrameEvent& evt) {
    // TODO: HANDLE JOINT LIMITS TOO!!!
    if(_rotation_delta != 0) {
        _fk->rotate(_selected_rotation_frame, _rotation_delta * evt.timeSinceLastFrame);
    
        for(intmax_t i = 0; i < 6; i++) {
            _robot_view->set_transform(i, _fk->get_transform(i));
            _collision_view->set_transform(i, _fk->get_transform(i));

            // Update the target end effector view
            // HACK: Transforming the coordinate system
            auto position = _fk->get_tcp_position();
            _end_effector_view->set_position(
                position[0],
                position[1],
                position[2]
            );

            auto orientation = _fk->get_tcp_orientation();
            _end_effector_view->set_orientation(
                orientation[0],
                orientation[1],
                orientation[2],
                orientation[3]
            );
        }
    }
}