#include "array"
#include "string"

#include "Ogre.h"
#include "OgreApplicationContext.h"

#include "axes_helper_factory.h"

//Ogre::SceneManager* AxisHelperFactory::_scene_manager {nullptr};

using namespace std;

Ogre::ManualObject* AxisHelperFactory::_get_axis_object(
    string name,
    array<real_t, 3> color,
    array<real_t, 3> direction
) {
    // Create world axes and attach them to the scene
    Ogre::ManualObject* axis;

    if(!_scene_manager->hasManualObject(name)) {

        axis = _scene_manager->createManualObject(name); 

        Ogre::MaterialPtr axis_material = 
            Ogre::MaterialManager::getSingleton().create(name + "_material"s, "General"s); 
            
        axis_material->setReceiveShadows(false); 
        axis_material->getTechnique(0)->setLightingEnabled(true); 
        axis_material->getTechnique(0)->getPass(0)->setDiffuse(color[0], color[1], color[2], 0); 
        axis_material->getTechnique(0)->getPass(0)->setAmbient(color[0], color[1], color[2]); 
        axis_material->getTechnique(0)->getPass(0)->setSelfIllumination(color[0], color[1], color[2]); 

        axis->begin(name + "_material"s, Ogre::RenderOperation::OT_LINE_LIST);
        axis->position(0, 0, 0);
        axis->position(direction[0], direction[1], direction[2]);
        axis->end(); 

    } else {
        axis = _scene_manager->getManualObject(name); 
    }

    return axis;
}

AxisHelperFactory::AxisHelperFactory(Ogre::SceneManager* scene_manager) :
    _scene_manager(scene_manager)
{}

Ogre::SceneNode* AxisHelperFactory::get_axes_node(string name) {
    
    // Return detached scene node if it already exists
    Ogre::SceneNode* axes_group {nullptr};
    
    if(_scene_manager->hasSceneNode(name))
    {
        // Get node
        axes_group = _scene_manager->getSceneNode(name);

        // Detach node
        axes_group->getParent()->removeChild(axes_group);
    }
    else {
        // Create axes group
        axes_group =
            _scene_manager->getRootSceneNode()->createChildSceneNode(name);

        // Create X axis
        Ogre::ManualObject* x_axis_object =
            _get_axis_object(name + "_x_axis", {1, 0, 0}, {1, 0, 0});
        
        Ogre::SceneNode* x_axis_node =
            axes_group->createChildSceneNode(name + "_x_axis_node"s); 
        
        x_axis_node->attachObject(x_axis_object);

        // Create Y axis
        Ogre::ManualObject* y_axis_object =
            _get_axis_object(name + "_y_axis", {0, 1, 0}, {0, 1, 0});
        Ogre::SceneNode* y_axis_node =
            axes_group->createChildSceneNode(name + "_y_axis_node"s); 
        
        y_axis_node->attachObject(y_axis_object);

        // Create Z axis
        Ogre::ManualObject* z_axis_object =
            _get_axis_object(name + "_z_axis", {0, 0, 1}, {0, 0, 1});
        Ogre::SceneNode* z_axis_node =
            axes_group->createChildSceneNode(name + "_z_axis_node"s); 
        
        z_axis_node->attachObject(z_axis_object);

        // Detach node
        axes_group->getParent()->removeChild(axes_group);
    }

    return axes_group;
}
