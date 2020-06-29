#include <array>

#include "robot_view.h"

using namespace std;

void RobotView::_create_base(Ogre::SceneNode* parent) {
    Ogre::SceneNode* base_node = parent->createChildSceneNode("base"s);

    Ogre::Entity* base_entity = _scene_manager->createEntity("base.mesh");
    base_node->attachObject(base_entity);
}

Ogre::SceneNode* RobotView::_create_joint(
    Ogre::SceneNode* parent,
    const std::string& name,
    intmax_t number_of_meshes
) {
    // Geometry entities
    Ogre::SceneNode* frame = parent->createChildSceneNode(name + "_frame");

    for(size_t i = 0; i < number_of_meshes; i++) {
        Ogre::Entity* submesh = _scene_manager->createEntity(name + to_string(i + 1) + ".mesh");
        Ogre::SceneNode* submesh_node = frame->createChildSceneNode(name + to_string(i + 1) );
        submesh_node->attachObject(submesh);
    }

    return frame;
}

RobotView::RobotView (
    Ogre::SceneManager* scene_manager
) :
    _scene_manager(scene_manager)
{
    array<string, 6> mesh_names {{
        "shoulder"s,
        "upperarm"s,
        "forearm"s,
        "wrist1_"s,
        "wrist2_"s,
        "wrist3_"s
    }};

    array<intmax_t, 6> mesh_count {{ 3, 4, 5, 3, 3, 1 }};

    _robot_view_group = _scene_manager->getRootSceneNode()->createChildSceneNode("robot_view");

    _create_base(_robot_view_group);
    
    for(intmax_t i = 0; i < 6; i++) {
        _joint_nodes[i] = _create_joint(
            _robot_view_group,
            mesh_names[i],
            mesh_count[i]
        );
    }
}

void RobotView::set_transform(size_t joint_index, const fcl::Transform3d& transform) {
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
