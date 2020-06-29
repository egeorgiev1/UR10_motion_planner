#pragma once

#include <array>
#include <string>

#include "Ogre.h"
#include "OgreApplicationContext.h" // IS THIS NEEDED???

#include "util.h"

class AxisHelperFactory {
private:
    Ogre::SceneManager* _scene_manager { nullptr };

    Ogre::ManualObject* _get_axis_object(
        std::string name,
        std::array<real_t, 3> color,
        std::array<real_t, 3> direction
    );

public:
    AxisHelperFactory(Ogre::SceneManager* scene_manager);

    Ogre::SceneNode* get_axes_node(std::string name);
};