#include "input_manager.h"

InputManager::InputManager() {}

void InputManager::add_listener (OgreBites::InputListener* listener) {
    _listeners.push_back(listener);
}

bool InputManager::keyPressed (const OgreBites::KeyboardEvent& evt) {
    for(OgreBites::InputListener *const listener: _listeners) {
        listener->keyPressed(evt);
    }
}

void InputManager::frameRendered (const Ogre::FrameEvent& evt) {
    for(OgreBites::InputListener *const listener: _listeners) {
        listener->frameRendered(evt);
    }
}

bool InputManager::keyReleased (const OgreBites::KeyboardEvent& evt) {
    for(OgreBites::InputListener *const listener: _listeners) {
        listener->keyReleased(evt);
    }
}

bool InputManager::mouseMoved (const OgreBites::MouseMotionEvent& evt) {
    for(OgreBites::InputListener *const listener: _listeners) {
        listener->mouseMoved(evt);
    }
}

bool InputManager::mousePressed (const OgreBites::MouseButtonEvent& evt) {
    for(OgreBites::InputListener *const listener: _listeners) {
        listener->mousePressed(evt);
    }
}

bool InputManager::mouseReleased (const OgreBites::MouseButtonEvent& evt) {
    for(OgreBites::InputListener *const listener: _listeners) {
        listener->mouseReleased(evt);
    }
}

bool InputManager::mouseWheelRolled (const OgreBites::MouseWheelEvent& evt) {
    for(OgreBites::InputListener *const listener: _listeners) {
        listener->mouseWheelRolled(evt);
    }
}

bool InputManager::touchMoved (const OgreBites::TouchFingerEvent& evt) {
    for(OgreBites::InputListener *const listener: _listeners) {
        listener->touchMoved(evt);
    }
}

bool InputManager::touchPressed (const OgreBites::TouchFingerEvent& evt) {
    for(OgreBites::InputListener *const listener: _listeners) {
        listener->touchPressed(evt);
    }
}

bool InputManager::touchReleased (const OgreBites::TouchFingerEvent& evt) {
    for(OgreBites::InputListener *const listener: _listeners) {
        listener->touchReleased(evt);
    }
}
