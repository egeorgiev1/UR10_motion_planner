#pragma once

#include <vector>
#include <string>

#include <Ogre.h>
#include <OgreApplicationContext.h>
#include <fcl/fcl.h>

#include "forward_kinematics.h"
#include "collision_selection_manager.h"
#include "util.h"

class RobotModel {
private:
	Ogre::SceneManager* _scene_manager { nullptr };
	fcl::BroadPhaseCollisionManager<double>* _collision_manager { nullptr };
    ForwardKinematics* _fk { nullptr };

	std::vector<real_t> _joint_angles;
	std::vector<Ogre::SceneNode*> _joint_nodes; // who owns these SceneNode pointers? Can I make them shared_ptr instead of plain-old C pointers???

    // In the order corresponding to _joint_angles for transform updates!!!
    CollisionSelectionManager* _collision_selection_manager { nullptr };
    std::vector<fcl::CollisionObject<double>*> _collision_objects; // who owns these pointers? Can I make them shared_ptr?
    std::vector<Ogre::ManualObject*> _collision_entities; // used for toggling visibility!
    bool colliders_visible { true };
    bool visual_mesh_visible { true };

    struct CollisionObjectDescriptor {
        Ogre::ManualObject* collider_entity;
        fcl::CollisionObject<double>* collider;
    };

    CollisionObjectDescriptor _createCollisionObject(const std::string& object_name_param);

    // xyz position and rpy orientation
    Ogre::SceneNode* _createBase();

    Ogre::SceneNode* _createJoint(
        std::string mesh_name,
        size_t mesh_count,
        size_t joint_index
    );

public:
    void toggle_collider_visibility();

    void toggle_visual_mesh_visibility();

	RobotModel(
        Ogre::SceneManager* scene_manager,
        fcl::BroadPhaseCollisionManager<double>* collision_manager,
        CollisionSelectionManager* collision_selection_manager,
        ForwardKinematics* fk
    );

    std::vector<real_t> get_angles() const;

    // TODO: WRITE MY OWN VERSION: https://forums.ogre3d.org/viewtopic.php?t=53647
    static void _DestroyAllAttachedMovableObjects(Ogre::SceneNode* i_pSceneNode);

    template<typename T> // TODO: size must be 6! Remove this template and use array<> only!!!
    void set_angles(T angles) {
        for(size_t i = 0; i < 6; i++) {
            set_angle(i, angles[i]);
        }

        update_collider_transforms();
    }

    // NOTE: Do for all joints, because one angle migth affect more than one joint! Optimization
    // for reducing the updates could be done, but most of the time with IK we'll update all angles
    // at once
    void update_collider_transforms();

	void set_angle(size_t joint_index, real_t angle);
	
	void rotate_joint(size_t joint_index, real_t angle);
	
};