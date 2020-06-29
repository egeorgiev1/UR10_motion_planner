#include "test_fcl_utility.h"

#include "collision_view.h"

using namespace std;

Ogre::ManualObject* CollisionView::_get_mesh(const string& name) {
    std::vector<fcl::Vector3d> vertices;
    std::vector<fcl::Triangle> triangles;
    
    // Fetch and parse .obj file
    string object_path = "../models/collision/"s + name + ".obj"s;
    fcl::test::loadOBJFile<double>(object_path.c_str(), vertices, triangles);

    // Ogre3D mesh generation test
    Ogre::ManualObject* collider_mesh = _scene_manager->createManualObject(name + "_collider"s);

    // Create wireframe material for this mesh
    Ogre::MaterialPtr wireframe_material =
        Ogre::MaterialManager::getSingleton().create(
            name + "_collider_material"s,
            Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME
        )
    ;
    wireframe_material->setDiffuse(Ogre::ColourValue(1, 1, 1));
    wireframe_material->setSelfIllumination(Ogre::ColourValue(1, 1, 1));
    wireframe_material->getTechnique(0)->getPass(0)->setPolygonMode(Ogre::PM_WIREFRAME);
    wireframe_material->setCullingMode(Ogre::CullingMode::CULL_NONE); // Two-sided rendering

    // Start setting up the mesh vertices and triangles
    collider_mesh->begin(name + "_collider_material"s, Ogre::RenderOperation::OT_TRIANGLE_LIST);

    // Go over all verticies and add them in order to the ManualObject's buffer
    for(size_t i = 0; i < vertices.size(); i++) {
        collider_mesh->position(
            vertices[i](0),
            vertices[i](1),
            vertices[i](2)
        );
    }

    // Go over all triangles
    for(size_t i = 0; i < triangles.size(); i++) {
        collider_mesh->triangle(
            triangles[i][0],
            triangles[i][1],
            triangles[i][2]
        );
    }

    collider_mesh->end();

    return collider_mesh;
}

Ogre::SceneNode* CollisionView::_create_node(
    Ogre::SceneNode* parent,
    const string& name
) {
    Ogre::SceneNode* node = parent->createChildSceneNode(name + "_collider"s);

    node->attachObject(
        _get_mesh(name)
    );

    return node;
}

CollisionView::CollisionView (
    Ogre::SceneManager* scene_manager
) :
    _scene_manager(scene_manager)
{
    array<string, 6> mesh_names {{
        "shoulder"s,
        "upperarm"s,
        "forearm"s,
        "wrist1"s,
        "wrist2"s,
        "wrist3"s
    }};

    _collision_view_group =
        _scene_manager->getRootSceneNode()->createChildSceneNode("collision_view"s);

    _create_node(_collision_view_group, "base"s);
    
    for(intmax_t i = 0; i < 6; i++) {
        _joint_nodes[i] = _create_node(
            _collision_view_group,
            mesh_names[i]
        );
    }
}

void CollisionView::set_transform(size_t joint_index, const fcl::Transform3d& transform) {
    // Get DH frame and decompose into position and quaternion
    fcl::Vector3d position = transform.translation();
    fcl::Matrix3d rotation_matrix = transform.rotation();
    fcl::Quaterniond rotation = fcl::Quaterniond(rotation_matrix);

    _joint_nodes[joint_index]->resetToInitialState();

    // Apply rotation first
    _joint_nodes[joint_index]->rotate(
        Ogre::Quaternion(
            rotation.w(),
            rotation.x(),
            rotation.y(),
            rotation.z()
        )
    );
    
    // Apply translation
    _joint_nodes[joint_index]->setPosition(
        Ogre::Vector3(
            position[0],
            position[1],
            position[2]
        )
    );
}

void CollisionView::set_visibility(bool is_visible) {
    _collision_view_group->setVisible(is_visible, true);
}