#pragma once

#include <Ogre.h>

#include "axes_helper_factory.h"
#include "util.h"

// End effector visualization
class EndEffectorView // Use an IEndEffectorView interface???
{
private:
    Ogre::SceneManager* _scene_manager {nullptr};
    AxisHelperFactory*_axis_helper_factory {nullptr};
    
    Ogre::SceneNode* _end_effector {nullptr};
public:
    EndEffectorView(Ogre::SceneManager* scene_manager, AxisHelperFactory* axis_helper_factory);

    void set_position(real_t x, real_t y, real_t z);

    // Refactor using custom radian literals and radian wrappers??? Don't use the Ogre3D ones. It's
    // an external library to which I don't want to couple the interface.
    void set_orientation(real_t yaw, real_t pitch, real_t roll);

    void set_orientation(real_t w, real_t x, real_t y, real_t z);
};