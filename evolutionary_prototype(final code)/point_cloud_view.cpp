#include <string>

#include "point_cloud_view.h"

using namespace std;

PointCloudView::PointCloudView(
    std::string uuid,
    Ogre::SceneManager* scene_manager
) :
    _uuid(uuid),
    _scene_manager(scene_manager)
{}

// TODO: Decouple the object pool code
void PointCloudView::set_points(const points_array_t& points) {
    // Detach from scene and return spheres that are in-use to the pool
    for(Ogre::SceneNode* sphere: _in_use_spheres) {
        sphere->getParentSceneNode()->removeChild(sphere);
        _available_spheres.push_back(sphere);
    }

    _in_use_spheres.erase(begin(_in_use_spheres), end(_in_use_spheres));

    // Display given points
    for(const auto& point : points) {
        Ogre::SceneNode* sphere;

        if(not _available_spheres.empty()) {
            // Get sphere from pool and mark as in-use
            sphere = _available_spheres[_available_spheres.size()-1];
            _available_spheres.pop_back();
            _in_use_spheres.push_back(sphere);
        } else {
            // Create sphere entity and add as in-use
            Ogre::Entity* sphere_entity = _scene_manager->createEntity(Ogre::SceneManager::PT_SPHERE);
            
            const string material_name { "point_cloud_sphere_material"s };
            Ogre::MaterialPtr sphere_material =
                Ogre::MaterialManager::getSingleton().getByName(material_name);
            
            // Create material if it doesn't exist
            if(not sphere_material) {
                sphere_material =
                    Ogre::MaterialManager::getSingleton().create(
                        material_name,
                        Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME
                    )
                ;

                sphere_material->setDiffuse(Ogre::ColourValue(0, 0, 1));
                sphere_material->setAmbient(Ogre::ColourValue(0, 0, 1)); 
            }

            sphere_entity->setMaterial(sphere_material);
            
            // Create sphere node
            sphere = _scene_manager->getRootSceneNode()->createChildSceneNode
            (
                    "point_cloud_sphere_node_"s +
                    // This is to support multiple PointCloudView instances(better to use random UUID)
                    _uuid + "_"s +
                    to_string(_available_spheres.size() + _in_use_spheres.size())
                )
            ;
            sphere->setScale(Ogre::Vector3(0.0002, 0.0002, 0.0002));
            sphere->attachObject(sphere_entity);

            // Detach from parent
            sphere->getParentSceneNode()->removeChild(sphere);

            // Add to in_use spheres
            _in_use_spheres.push_back(sphere);
        }

        // Set the point's position
        sphere->setPosition(Ogre::Vector3(
            point[0],
            point[1],
            point[2]
        ));

        // Add to the scene
        _scene_manager->getRootSceneNode()->addChild(sphere);
    }
}


void PointCloudView::set_visibility(bool is_visible) {
    for(auto scene_node : _available_spheres) {
        scene_node->setVisible(is_visible, true);
    }

    for(auto scene_node : _in_use_spheres) {
        scene_node->setVisible(is_visible, true);
    }
}