#pragma once

#include <Ogre.h>
#include <OgreApplicationContext.h>
#include <OgreCameraMan.h>
#include "input_manager.h"

class ObjectSelectionController : public OgreBites::InputListener {
private:
    Ogre::SceneManager*   _scene_manager    { nullptr };
    Ogre::Camera*         _camera           { nullptr };
    OgreBites::CameraMan* _orbit_controller { nullptr };
    Ogre::RenderWindow*   _render_window    { nullptr };
    InputManager*         _input_manager    { nullptr };

    Ogre::SceneNode* _last_selection { nullptr };

    bool is_selection_mode_enabled { false };

    void _select_object(Ogre::SceneNode* node);

    void _select_object_through_viewport_raycast(
        decltype(OgreBites::MouseButtonEvent::x) x,
        decltype(OgreBites::MouseButtonEvent::y) y
    );
public:
    ObjectSelectionController(
        Ogre::SceneManager* scene_manager,
        Ogre::Camera* camera,
        OgreBites::CameraMan* orbit_controller,
        Ogre::RenderWindow* render_window,
        InputManager* input_manager
    );

    // Toggle the different axes for position/orientation
	bool keyPressed(const OgreBites::KeyboardEvent& evt) override;
    
    // Toggle the different axes for position/orientation
	bool keyReleased(const OgreBites::KeyboardEvent& evt) override;

    bool mousePressed(const OgreBites::MouseButtonEvent& evt) override;

    // bool mouseMoved(const OgreBites::MouseMotionEvent &evt) override;
};