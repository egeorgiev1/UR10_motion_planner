#pragma once

#include <array>
#include <string>
//#include <map>

#include "Ogre.h"
#include "fcl/fcl.h"
#include "test_fcl_utility.h"
#include "eigen3/Eigen/Eigen"

#include "forward_kinematics.h"
#include "util.h"

#include <iostream>
using namespace std;

class CollisionModel {
private:

	fcl::BroadPhaseCollisionManager<real_t>* _collision_manager { nullptr };

    fcl::CollisionObject<real_t>* _base_collider { nullptr };
    std::array<fcl::CollisionObject<real_t>*, 6> _joint_colliders;

    // Used in self_collision_function to avoid collision detection of adjacent meshes
    //std::map<fcl::CollisionObject<real_t>*, intmax_t> _collider_pointer_to_index;

    fcl::CollisionObject<real_t>* _get_collider(const std::string& name);

public:
    CollisionModel();

    // This function is friend to the class by default, hence you can access private data of collision_model
    // in this static function
    template <typename S>
    static bool self_collision_function(
        fcl::CollisionObject<S>* o1,
        fcl::CollisionObject<S>* o2,
        void* cdata_
    ) {
        auto* cdata = static_cast<fcl::test::CollisionData<S>*>(cdata_);
        const auto& request = cdata->request;
        auto& result = cdata->result;

        if(cdata->done) return true;

        // If o1 and o2 are the same meshes or adjacent, don't check for collision
        intmax_t index_diff = abs(
            (*reinterpret_cast<intmax_t*>(o1->getUserData())) -
            (*reinterpret_cast<intmax_t*>(o2->getUserData()))
        );

        // cout << "BLABLA" << endl;
        if(index_diff > 1) {
            collide(o1, o2, request, result);
        }

        if(!request.enable_cost && (result.isCollision()) && (result.numContacts() >= request.num_max_contacts))
            cdata->done = true;

        return cdata->done;
    }

    // THIS ONE IS USED FOR LOGGING PURPOSES!!!
    template <typename S>
    static bool scene_collision_function(
        fcl::CollisionObject<S>* o1,
        fcl::CollisionObject<S>* o2,
        void* cdata_
    ) {
        auto* cdata = static_cast<fcl::test::CollisionData<S>*>(cdata_);
        const auto& request = cdata->request;
        auto& result = cdata->result;

        if(cdata->done) return true;

        //cout << "BLABLA" << endl;

        collide(o1, o2, request, result);

        if(!request.enable_cost && (result.isCollision()) && (result.numContacts() >= request.num_max_contacts))
            cdata->done = true;

        //cout << cdata->done << endl;

        return cdata->done;
    }

    void set_transform(size_t joint_index, const fcl::Transform3d& transform);

    fcl::BroadPhaseCollisionManager<real_t>* get_broadphase_managers();
};
