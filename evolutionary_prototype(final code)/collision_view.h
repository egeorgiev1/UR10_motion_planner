#pragma once

#include <array>
#include <string>

#include "Ogre.h"
#include "fcl/fcl.h"
#include "eigen3/Eigen/Eigen"

#include "forward_kinematics.h"

class CollisionView {
private:
    Ogre::SceneManager* _scene_manager { nullptr };

    Ogre::SceneNode* _collision_view_group { nullptr };
    std::array<Ogre::SceneNode*, 6> _joint_nodes;

    Ogre::ManualObject* _get_mesh(const std::string& name);

    Ogre::SceneNode* _create_node(
        Ogre::SceneNode* parent,
        const std::string& name
    );

public:
    CollisionView(
        Ogre::SceneManager* scene_manager
    );

    void set_transform(size_t joint_index, const fcl::Transform3d& transform);

    void set_visibility(bool is_visible);
};
