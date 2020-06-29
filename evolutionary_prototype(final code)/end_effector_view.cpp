#include <iostream>
#include "end_effector_view.h"

using namespace std;

EndEffectorView::EndEffectorView(Ogre::SceneManager* scene_manager, AxisHelperFactory* axis_helper_factory)
:
    _scene_manager(scene_manager),
    _axis_helper_factory(axis_helper_factory)
{
    // Create end effector group node
    _end_effector = _scene_manager->getRootSceneNode()->createChildSceneNode("end_effector");

    // Create red sphere entity
    Ogre::Entity* sphere_entity = _scene_manager->createEntity(Ogre::SceneManager::PT_SPHERE); // Use a weak_ptr??? Who will manage the deletion of this object???
    sphere_entity->getSubEntity(0)->getMaterial()->setDiffuse(Ogre::ColourValue(1, 0, 0));
    sphere_entity->getSubEntity(0)->getMaterial()->setAmbient(Ogre::ColourValue(1, 0, 0)); // Derive from diffuse???
    
    // Scale sphere node properly
    Ogre::SceneNode* sphere_node =
        _scene_manager->getRootSceneNode()->createChildSceneNode("sphere_node");

    sphere_node->setScale(Ogre::Vector3(0.0002, 0.0002, 0.0002));
    sphere_node->setPosition(Ogre::Vector3(0, 0, 0)); // TODO: Is this needed???

    sphere_node->attachObject(sphere_entity);
    sphere_node->getParentSceneNode()->removeChild(sphere_node);

    // Create local axes for visualizing end effector orientation
    Ogre::SceneNode* axes_helper_node = _axis_helper_factory->get_axes_node("end_effector_axes"s);
    axes_helper_node->setPosition(0.0001, 0, 0); // Improvement on flickering issue from interleaving with the grid. POSSIBLE SOLUTION: use cylinders instead of lines
    axes_helper_node->setScale(0.1, 0.1, 0.1);

    // Attach the sphere and the axes visualization to the end effector node
    _end_effector->addChild(sphere_node);
    _end_effector->addChild(axes_helper_node);
}

void EndEffectorView::set_position(real_t x, real_t y, real_t z) {
    _end_effector->setPosition( {x, y, z} ); // Will this work???

    // DEBUG
    cout << "END EFFECTOR POSITION SET:" << endl;
    cout << "x: " << x << endl;
    cout << "y: " << y << endl;
    cout << "z: " << z << endl;
    cout << endl;
}

// Refactor using custom radian literals and radian wrappers??? Don't use the Ogre3D ones. It's
// an external library to which I don't want to couple the interface.
void EndEffectorView::set_orientation(real_t yaw, real_t pitch, real_t roll) {
    _end_effector->resetOrientation();

    // TODO: TEMPORARILY FIXING THE AXES!!! UNDO THIS LATER!!!
    _end_effector->yaw( Ogre::Radian{ roll } );
    _end_effector->pitch( Ogre::Radian{ -yaw } );
    _end_effector->roll( Ogre::Radian{ pitch } );

    // DEBUG
    cout << "END EFFECTOR ORIENTATION SET:" << endl;
    cout << "Yaw: " << yaw << endl;
    cout << "Pitch: " << pitch << endl;
    cout << "Roll: " << roll << endl;
    cout << endl;
}

void EndEffectorView::set_orientation(real_t w, real_t x, real_t y, real_t z) {
    _end_effector->resetOrientation();

    _end_effector->setOrientation(w, x, y, z);

    // DEBUG
    cout << "END EFFECTOR ORIENTATION SET:" << endl;
    cout << "w: " << w << endl;
    cout << "x: " << x << endl;
    cout << "y: " << y << endl;
    cout << "z: " << z << endl;
    cout << endl;
}
