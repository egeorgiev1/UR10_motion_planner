#pragma once

#include <vector>

#include <Ogre.h>
#include <OgreApplicationContext.h>

// Objects that implement OgreBites::InputListener can
// register their input handlers to an InputManager object.
// This must be done only by controller objects!
class InputManager : public OgreBites::InputListener {
private:
    std::vector<OgreBites::InputListener*> _listeners;
public:
    InputManager();

    void add_listener(OgreBites::InputListener* listener);
    bool keyPressed(const OgreBites::KeyboardEvent& evt) override;
    void frameRendered (const Ogre::FrameEvent& evt) override;
    bool keyReleased (const OgreBites::KeyboardEvent& evt) override;
    bool mouseMoved (const OgreBites::MouseMotionEvent& evt) override;
    bool mousePressed (const OgreBites::MouseButtonEvent& evt) override;
    bool mouseReleased (const OgreBites::MouseButtonEvent& evt) override;
    bool mouseWheelRolled (const OgreBites::MouseWheelEvent& evt) override;
    bool touchMoved (const OgreBites::TouchFingerEvent& evt) override;
    bool touchPressed (const OgreBites::TouchFingerEvent& evt) override;
    bool touchReleased (const OgreBites::TouchFingerEvent& evt) override;
};