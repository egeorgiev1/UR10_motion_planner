#pragma once

#include <string>
#include <array>

#include <Ogre.h>
#include <fcl/fcl.h>

#include "util.h"

class BoxCollider {
private:
	std::string _uuid;

	Ogre::SceneManager* _scene_manager;
	fcl::BroadPhaseCollisionManager<real_t>* _scene_collision_manager;

	fcl::CollisionObject<real_t>* _create_collision_object(
		const std::vector<fcl::Vector3d>& vertices,
		const std::vector<fcl::Triangle>& triangles
	);

	Ogre::ManualObject* _create_manual_object(
		const std::vector<fcl::Vector3d>& vertices,
		const std::vector<fcl::Triangle>& triangles
	);

	Ogre::SceneNode* _create_scene_node(Ogre::ManualObject* manual_object);
public:
	BoxCollider(
		std::string uuid,
		Ogre::SceneManager* scene_manager,
		fcl::BroadPhaseCollisionManager<real_t>* scene_collision_manager,
		std::array<real_t, 3> size,
		std::array<real_t, 3> position,
		std::array<real_t, 4> orientation // quaternion
	);
};
