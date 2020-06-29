#pragma once

#include <array>
#include <string>

#include "Ogre.h"
#include "fcl/fcl.h"
#include "eigen3/Eigen/Eigen"

#include "forward_kinematics.h"

class RobotView {
private:
    Ogre::SceneManager* _scene_manager { nullptr };

    Ogre::SceneNode* _robot_view_group { nullptr };
    std::array<Ogre::SceneNode*, 6> _joint_nodes;

    void _create_base(Ogre::SceneNode* parent);
    Ogre::SceneNode* _create_joint(
        Ogre::SceneNode* parent,
        const std::string& name,
        intmax_t number_of_meshes
    );
public:
    RobotView(
        Ogre::SceneManager* scene_manager
    );

    void set_transform(size_t joint_index, const fcl::Transform3d& transform);
};
