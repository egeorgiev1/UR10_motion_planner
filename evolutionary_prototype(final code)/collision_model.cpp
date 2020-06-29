#include "collision_model.h"

using namespace std;

fcl::CollisionObject<real_t>* CollisionModel::_get_collider(const string& name) {
    std::vector<fcl::Vector3d> vertices;
    std::vector<fcl::Triangle> triangles;
    
    // Fetch and parse .obj file
    string object_path = "../models/collision/"s + name + ".obj"s;
    fcl::test::loadOBJFile<real_t>(object_path.c_str(), vertices, triangles);
    
    // Create and populate bounding volume hierarchy of the mesh data
    typedef fcl::BVHModel<fcl::OBBRSSd> bvh_model_t; // TODO: Change to fcl::OBBRSS<real_t>
    shared_ptr<bvh_model_t> model = make_shared<bvh_model_t>();
    
    // Add mesh data to BVH
    model->beginModel();
    model->addSubModel(vertices, triangles);
    model->endModel();

    // Create a collision object from the BVH
    fcl::CollisionObject<real_t>* collider = new fcl::CollisionObject<real_t>{ model };

    return collider;
}

CollisionModel::CollisionModel()
{
    // Create collision manager (TODO: replace with AABB broadphase manager with a spatial data structure???)
    _collision_manager = new fcl::NaiveCollisionManager<real_t>{};

    // Create colliders and put them in the broadphase managers(no adjacent
    // links in a single manager)
    array<string, 6> mesh_names {{
        "shoulder"s,
        "upperarm"s,
        "forearm"s,
        "wrist1"s,
        "wrist2"s,
        "wrist3"s
    }};

    // Create and register base
    _base_collider = _get_collider("base"s);

    _collision_manager->registerObject(
        _base_collider
    );

    // Add index as userData to be used to avoid collisions
    // with adjacent objects
    _base_collider->setUserData(new intmax_t {0});

    // Map pointer to index
    //_collider_pointer_to_index[_base_collider] = 0;
    
    // Register joint link collision meshes
    for(intmax_t i = 0; i < 6; i++) {
        _joint_colliders[i] = _get_collider(
            mesh_names[i]
        );

        // Register collider
        _collision_manager->registerObject(
            _joint_colliders[i]
        );

        // Map pointer to index
        //_collider_pointer_to_index[_joint_colliders[i]] = i + 1;

        // Add index as userData to be used to avoid collisions
        // with adjacent objects
        _joint_colliders[i]->setUserData(new intmax_t {i + 1});
    }
}

void CollisionModel::set_transform(size_t joint_index, const fcl::Transform3d& transform) {

    _joint_colliders[joint_index]->setTransform( transform );

    /*
        In this example we can observe that they call computeAABB every time they change
        CollisionObject's transform:
        https://github.com/flexible-collision-library/fcl/blob/9dba579158109c0164bfe0e8b4a75c16bfc788f6/test/test_fcl_broadphase_collision_1.cpp

        Broadphase with AABB can work without calling computeAABB, but it will be slower:
        https://github.com/flexible-collision-library/fcl/issues/40
    */
    _joint_colliders[joint_index]->computeAABB();
}

fcl::BroadPhaseCollisionManager<real_t>* CollisionModel::get_broadphase_managers() { // TODO: Change to singular instead of plural
    // Return a copy of the array of broadphase managers
    return _collision_manager;
}