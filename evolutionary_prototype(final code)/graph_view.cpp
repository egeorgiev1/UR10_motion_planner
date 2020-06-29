#include "graph_view.h"

Ogre::ManualObject* GraphView::_create_graph_geometry() {
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
    
    // Create ManualObject
    Ogre::ManualObject* manual_object = _scene_manager->createManualObject(_uuid);

    return manual_object;
}

GraphView::GraphView(
    std::string uuid,
    Ogre::SceneManager* scene_manager,
    Ogre::ColourValue edge_color
) :
    _uuid(uuid),
    _scene_manager(scene_manager),
    _edge_color(edge_color)
{
    _graph_geometry = _create_graph_geometry();

    _scene_node = _scene_manager->getRootSceneNode()->createChildSceneNode(_uuid);
    _scene_node->attachObject(_graph_geometry);
}

void GraphView::set_edges(const std::vector<GraphEdge>& edge_list) {

    _graph_geometry->clear();

    // Start setting up the geometry
    _graph_geometry->begin(
        _uuid,
        Ogre::RenderOperation::OT_LINE_LIST
    );

    for(const GraphEdge& edge : edge_list) {
        _graph_geometry->position(Ogre::Vector3{
            edge.start[0],
            edge.start[1],
            edge.start[2]
        });

        _graph_geometry->position(Ogre::Vector3{
            edge.end[0],
            edge.end[1],
            edge.end[2]
        });

        // NOTE: Use _graph_geometry->index()???

        
    }

    // _graph_geometry->index(0);
    // _graph_geometry->index(1);

    // _graph_geometry->index(2);
    // _graph_geometry->index(3);

    _graph_geometry->end();
}


void GraphView::set_visibility(bool is_visible) {
    _scene_node->setVisible(is_visible, true);
}