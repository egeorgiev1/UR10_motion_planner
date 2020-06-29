#include "trajectory_controller.h"

#include <vector>
#include <iostream>

using namespace std;

void TrajectoryController::_handle_key_press_and_release(
    const OgreBites::KeyboardEvent& evt,
    bool is_pressed
) {
    // Apply delta if pressed, else reset delta
    if(is_pressed) {
        // Handle delta
        switch(evt.keysym.sym) {
            case ',': {
                _step_delta = -_step_speed;
                break;
            }
            case '.': {
                _step_delta = _step_speed;
                break;
            }
            default: {
                break;
            }
        }
    } else {
        _step_delta = 0;
    }
}

TrajectoryController::TrajectoryController(
    ForwardKinematics* fk,
    RobotView* robot_view,
    CollisionView* collision_view,
    CollisionModel* collision_model,
    CollisionController* collision_controller,
    EndEffectorView* end_effector_view,
    InputManager* input_manager,
    real_t step_speed
) :
    _fk(fk),
    _robot_view(robot_view),
    _collision_view(collision_view),
    _collision_model(collision_model),
    _collision_controller(collision_controller),
    _end_effector_view(end_effector_view),
    _input_manager(input_manager),
    _step_speed(step_speed)
{
    _input_manager->add_listener(this);

    cout << "INITIALIZED TRAJECTORY CONTROLLER" << endl;
}

// Toggle the different axes for position/orientation
bool TrajectoryController::keyPressed(const OgreBites::KeyboardEvent& evt) {
    _handle_key_press_and_release(evt, true);
}

// Toggle the different axes for position/orientation
bool TrajectoryController::keyReleased(const OgreBites::KeyboardEvent& evt) {
    _handle_key_press_and_release(evt, false);
}

// Increment the position/orientation axes using the deltas on every
// frameRendered event. If the position or orientation is updated, because
// the delta is different from zero then recalculate the IK. 
void TrajectoryController::frameRendered (const Ogre::FrameEvent& evt)
{    
    _current_step = _current_step + _step_delta;

    intmax_t current_step_index { floor(_current_step) };

    // Clamp _current_step
    if(_current_step < 0)
    {
        _current_step = 0;
        current_step_index = 0;
    }
    else if(_current_step >= _trajectory.size())
    {
        _current_step = _trajectory.size() - 1;
        current_step_index = _trajectory.size() - 1;
    }

    if(_step_delta && _trajectory.size() > 0) {

        // Apply joint angles
        _fk->set_angles(_trajectory[current_step_index].joint_angles);

        for(intmax_t i = 0; i < 6; i++) {
            _robot_view->set_transform(i, _fk->get_transform(i));
            _collision_view->set_transform(i, _fk->get_transform(i));
            _collision_model->set_transform(i, _fk->get_transform(i));
            _collision_controller->schedule_collision_detection();
        }

        // Update the target end effector view
        // HACK: Transforming the coordinate system
        _end_effector_view->set_position(
            -(_trajectory[current_step_index].position[0]),
            _trajectory[current_step_index].position[2],
            _trajectory[current_step_index].position[1]
        );

        _end_effector_view->set_orientation(
            _trajectory[current_step_index].orientation[0],
            _trajectory[current_step_index].orientation[1],
            _trajectory[current_step_index].orientation[2],
            _trajectory[current_step_index].orientation[3]
        );
    }
}

void TrajectoryController::set_trajectory(const trajectory_t& trajectory) {
    _trajectory = trajectory;
}