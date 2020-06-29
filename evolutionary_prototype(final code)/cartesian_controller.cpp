#include "cartesian_controller.h"

#include <vector>
#include <iostream>

using namespace std;

void CartesianController::_handle_key_press_and_release(
    const OgreBites::KeyboardEvent& evt,
    bool is_pressed
) {
    const decltype(evt.keysym.mod) MOD_CTRL = 1; // TODO: DA NIAMAM PREDVID SHIFT??? vaobshte togava nuzno li e da definevam tova???

    real_t delta = 0.0;
    array<real_t, 3>* array_to_be_modified = nullptr;

    // Determine if the position or the orientation delta
    // needs to be modified
    if(evt.keysym.mod == 0) {
        array_to_be_modified = &_delta_position;
    } else if(evt.keysym.mod == MOD_CTRL) {
        array_to_be_modified = &_delta_orientation;
    }

    // Apply delta if pressed, else remove delta
    if(is_pressed) {
        if(evt.keysym.mod == 0) {
            delta = 0.1;
        } else if(evt.keysym.mod == MOD_CTRL) {
            delta = 0.4;
        }
    } else {
        delta = 0.0;
    }

    // NOTE: If Shift is released before W, A, S or D then the end effector will
    // continue to rotate
    // NOTE: If Shift is pressed after holding any of the W, A, S or D key and then the
    // W, A, S and D keys start to get released then the end effector will conitue
    // to change its position
    // TODO: A more robust system needs to be made that will generate a /pressed/ and /released/
    // event for keyboard combinations and even take into account order of pressing using a stack

    // Apply the delta to the corresponding axis
    if(array_to_be_modified != nullptr) {
        switch(evt.keysym.sym) {
            case 's': {
                (*array_to_be_modified)[0] = delta;
                break;
            }
            case 'w': {
                (*array_to_be_modified)[0] = -delta;
                break;
            }
            case 'a': {
                (*array_to_be_modified)[1] = -delta;
                break;
            }
            case 'd': {
                (*array_to_be_modified)[1] = delta;
                break;
            }
            case 'q': {
                (*array_to_be_modified)[2] = -delta;
                break;
            }
            case 'e': {
                (*array_to_be_modified)[2] = delta;
                break;
            }
            default: {
                break;
            }
        }
    }
}

bool CartesianController::_is_delta_zero() {
    if(
        _delta_position[0] == 0 &&
        _delta_position[1] == 0 &&
        _delta_position[2] == 0 &&

        _delta_orientation[0] == 0 &&
        _delta_orientation[1] == 0 &&
        _delta_orientation[2] == 0
    ) {
        return true;
    } else {
        return false;
    }
}

CartesianController::CartesianController(
    ForwardKinematics* fk,
    InverseKinematics* ik_solver,
    RobotView* robot_view,
    CollisionView* collision_view,
    CollisionModel* collision_model,
    CollisionController* collision_controller,
    EndEffectorView* end_effector_view,
    InputManager* input_manager
) :
    _fk(fk),
    _ik_solver(ik_solver),
    _robot_view(robot_view),
    _collision_view(collision_view),
    _collision_model(collision_model),
    _collision_controller(collision_controller),
    _end_effector_view(end_effector_view),
    _input_manager(input_manager)
{
    _input_manager->add_listener(this);

    cout << "INITIALIZED CARTESIAN CONTROLLER" << endl;
}

std::array<real_t, 6> CartesianController::get_angles() {
    return _fk->get_angles();
}

// Toggle the different axes for position/orientation
bool CartesianController::keyPressed(const OgreBites::KeyboardEvent& evt) {
    _handle_key_press_and_release(evt, true);
}

// Toggle the different axes for position/orientation
bool CartesianController::keyReleased(const OgreBites::KeyboardEvent& evt) {
    _handle_key_press_and_release(evt, false);
}

// Increment the position/orientation axes using the deltas on every
// frameRendered event. If the position or orientation is updated, because
// the delta is different from zero then recalculate the IK. 
void CartesianController::frameRendered (const Ogre::FrameEvent& evt) {
    // TODO: Use this: evt.timeSinceLastFrame ???

    // Apply delta, perform IK, update RobotModel and EndEffectorView
    if(!_is_delta_zero()) {

        // Apply delta
        _target_position[0] += _delta_position[0] * evt.timeSinceLastFrame;
        _target_position[1] += _delta_position[1] * evt.timeSinceLastFrame;
        _target_position[2] += _delta_position[2] * evt.timeSinceLastFrame;
        
        _target_orientation[0] += _delta_orientation[0] * evt.timeSinceLastFrame;
        _target_orientation[1] += _delta_orientation[1] * evt.timeSinceLastFrame;
        _target_orientation[2] += _delta_orientation[2] * evt.timeSinceLastFrame;

        // Perform IK
        _ik_solver->set_target_position(_target_position);
        _ik_solver->set_target_orientation(_target_orientation);

        // Limit the scope of variables when setting the current angles
        {
            // vector<real_t> current_angles = _fk->get_angles();
            // array<real_t, 6> current_angles_static_array {0, 0, 0, 0, 0, 0};
            array<real_t, 6> current_angles_static_array { _fk->get_angles() };

            // for(intmax_t i = 0; i < current_angles_static_array.size(); i++) {
            //     current_angles_static_array[i] = current_angles[i];
            // }

            _ik_solver->set_current_angles(current_angles_static_array);
        }

        array<real_t, 6> resulting_angles {0, 0, 0, 0, 0, 0};
        intmax_t error_code = _ik_solver->solve(resulting_angles);

        if(error_code == -1) {
            cout << _ik_solver->get_last_error() << endl;
        }

        if(error_code == 0) {
            // Apply IK joint angles
            _fk->set_angles(resulting_angles);

            // DEBUG
            cout << "IK Joint Angles: ";

            for(intmax_t i = 0; i < 6; i++) {
                cout << resulting_angles[i] << " ";
            }

            cout << endl;
    
            for(intmax_t i = 0; i < 6; i++) {
                _robot_view->set_transform(i, _fk->get_transform(i));
                _collision_view->set_transform(i, _fk->get_transform(i));
                _collision_model->set_transform(i, _fk->get_transform(i));
                _collision_controller->schedule_collision_detection();
            }
        }

        // Update the target end effector view
        // HACK: Transforming the coordinate system
        _end_effector_view->set_position(
            -(_target_position[0]),
            _target_position[2],
            _target_position[1]
        );

        // INCORRECT ORIENTATION!!!
        _end_effector_view->set_orientation(
            _target_orientation[0],
            _target_orientation[1],
            _target_orientation[2]
        );
    }
}