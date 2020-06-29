#include "object_selection_controller.h"

#include <iostream>

using namespace std;

void ObjectSelectionController::_select_object(Ogre::SceneNode* node) {
    if(_last_selection) {
        _last_selection->showBoundingBox(false);
    }

    node->showBoundingBox(true);
    _last_selection = node;
}

void ObjectSelectionController::_select_object_through_viewport_raycast(
    decltype(OgreBites::MouseButtonEvent::x) x,
    decltype(OgreBites::MouseButtonEvent::y) y
) {
    // Raycast from viewport
    Ogre::RaySceneQuery *mRaySceneQuery = _scene_manager->createRayQuery(Ogre::Ray());
    mRaySceneQuery->setSortByDistance( true );

    Ogre::Ray scopeRay = _camera->getCameraToViewportRay(
        static_cast<double>(x)/_render_window->getWidth(),
        static_cast<double>(y)/_render_window->getHeight()
    );

    mRaySceneQuery->setRay(scopeRay);
    Ogre::RaySceneQueryResult& result = mRaySceneQuery->execute();
    
    if(result.size()) {

        // All of the objects placed in the scene are movable for easier implementation. Optimizations with static geometry are not needed
        _orbit_controller->setTarget(
            dynamic_cast<Ogre::SceneNode*>(result.at(0).movable->getParentNode())
        );

        // Show bounding box for indicating the selection
        _select_object(
            dynamic_cast<Ogre::SceneNode*>(result.at(0).movable->getParentNode())
        );

        // Force camera position update
        OgreBites::MouseMotionEvent simulated_motion_event;
        simulated_motion_event.type = OgreBites::MOUSEMOTION;
        simulated_motion_event.x = x;
        simulated_motion_event.y = y;
        simulated_motion_event.xrel = 0;
        simulated_motion_event.yrel = 0;
        _orbit_controller->mouseMoved(simulated_motion_event);
        cout << "I AM EXECUTIN THIS!!!" << endl;

        cout << ("ACTION: Switched camera target to " + result.at(0).movable->getParentNode()->getName()) << endl;
    }
}

ObjectSelectionController::ObjectSelectionController(
    Ogre::SceneManager* scene_manager,
    Ogre::Camera* camera,
    OgreBites::CameraMan* orbit_controller,
    Ogre::RenderWindow* render_window,
    InputManager* input_manager
) :
    _scene_manager(scene_manager),
    _camera(camera),
    _orbit_controller(orbit_controller),
    _render_window(render_window),
    _input_manager(input_manager)
{
    _input_manager->add_listener(this);
}

// Toggle the different axes for position/orientation
bool ObjectSelectionController::keyPressed(const OgreBites::KeyboardEvent& evt) {
    if (evt.keysym.sym == 'x' && evt.keysym.mod == 0 && (bool)evt.repeat == false) {
        is_selection_mode_enabled = true;
        cout << "ACTION: Selection mode enabled" << endl;
    }
}

// Toggle the different axes for position/orientation
bool ObjectSelectionController::keyReleased(const OgreBites::KeyboardEvent& evt) {
    if (evt.keysym.sym == 'x' && evt.keysym.mod == 0 && (bool)evt.repeat == false) {
        is_selection_mode_enabled = false;
        cout << "ACTION: Selection mode disabled" << endl;
    }
}

bool ObjectSelectionController::mousePressed(const OgreBites::MouseButtonEvent& evt) {
    if(is_selection_mode_enabled) {
        _select_object_through_viewport_raycast(evt.x, evt.y);
    }

    return true;
}

// bool ObjectSelectionController::mouseMoved(const OgreBites::MouseMotionEvent &evt) { 

//     cout << "MOUSE MOVED DBEUG: " << endl;
//     cout << evt.type << endl;
//     cout << evt.windowID << endl;
//     cout << evt.x << endl;
//     cout << evt.xrel << endl;
//     cout << evt.y << endl;
//     cout << evt.yrel << endl;
//     cout << endl;

//     return true;
// }
