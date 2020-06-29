#include <OgreProcedural/Procedural.h>

#include "path_view.h"

Ogre::MaterialPtr PathView::_create_path_material() {
    // Create/reuse material
    const std::string material_name { _uuid };

    Ogre::MaterialPtr edge_material =
        Ogre::MaterialManager::getSingleton().getByName(material_name);
    
    if(not edge_material) {
        edge_material =
            Ogre::MaterialManager::getSingleton().create(
                material_name,
                Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME
            )
        ; 

        // Create wireframe material for this mesh(TODO: CHANGE FROM WIREFRAME TO SOLID MATERIAL?!)
        edge_material->setDiffuse(_edge_color);
        edge_material->setAmbient(_edge_color);
        edge_material->setSelfIllumination(_edge_color);
        //edge_material->setSelfIllumination(_edge_color);
        //edge_material->getTechnique(0)->getPass(0)->setPolygonMode(Ogre::PM_WIREFRAME);
        //edge_material->setCullingMode(Ogre::CullingMode::CULL_NONE); // Two-sided rendering
    }

    return edge_material;
}

PathView::PathView(
    std::string uuid,
    Ogre::SceneManager* scene_manager,
    Ogre::ColourValue edge_color
) :
    _uuid(uuid),
    _scene_manager(scene_manager),
    _edge_color(edge_color)
{
    _scene_node = _scene_manager->getRootSceneNode()->createChildSceneNode(_uuid);
}

void PathView::set_points(const std::vector<std::array<real_t, 3>>& points)
{
    if(points.empty()) {
        return;
    }
    
    // Use this for cleanup before recreating the object:
    // https://forums.ogre3d.org/viewtopic.php?t=63104
    if(_scene_node->numAttachedObjects())
    {
        _scene_node->detachObject(_uuid);
        _scene_manager->destroyEntity(_uuid);

        Ogre::MeshManager::getSingleton().getByName(_uuid)->unload();
        Ogre::MeshManager::getSingleton().remove(_uuid);
    }

    // Create path from the given points
    Procedural::Path path;

    for(const auto& point : points) {
        path.addPoint(
            point[0],
            point[1],
            point[2]
        );
    }

    // Extrude a small circle along the path
    Procedural::Shape circle = Procedural::CircleShape().setRadius(0.005).realizeShape();
    Procedural::Extruder().setExtrusionPath(&path).setShapeToExtrude(&circle).realizeMesh(_uuid);

    Ogre::Entity* path_entity = _scene_manager->createEntity(_uuid, _uuid);
    path_entity->setMaterial( _create_path_material() );
    _scene_node->attachObject(path_entity);
}
