#pragma once

#include <map>
#include <Ogre.h>
#include <fcl/fcl.h>

class CollisionSelectionManager { // pass this to every collision object creator!!!
private:
    // map from collisionobject pointer to scenenode pointer
    // currently selected object
    std::map<
        fcl::CollisionObject<double>*,
        Ogre::ManualObject*
    > _collision_object_to_collider_entity;
    
    fcl::CollisionObject<double>* _selection_first { nullptr };
    fcl::CollisionObject<double>* _selection_second { nullptr };
public:
    void add(fcl::CollisionObject<double>* collider, Ogre::ManualObject* collider_entity);

    // Select colliding pair of objects
    void select_pair(
        fcl::CollisionObject<double>* collider1,
        fcl::CollisionObject<double>* collider2
    );
};
