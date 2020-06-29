#include "collision_selection_manager.h"

void CollisionSelectionManager::add(
    fcl::CollisionObject<double>* collider,
    Ogre::ManualObject* collider_entity
) {
    _collision_object_to_collider_entity[collider] = collider_entity;
}

// Select colliding pair of objects
void CollisionSelectionManager::select_pair(
    fcl::CollisionObject<double>* collider1,
    fcl::CollisionObject<double>* collider2
) {
    // If collider is not in the data structures then throw an exception!!!
    if(
        _collision_object_to_collider_entity.find(collider1) ==
        _collision_object_to_collider_entity.end()
    ) {
        throw "collider1 not in CollisionSelectionManager!";
    }

    if(
        _collision_object_to_collider_entity.find(collider2) ==
        _collision_object_to_collider_entity.end()
    ) {
        throw "collider2 not in CollisionSelectionManager!";
    }

    // Set color back to white
    if(_selection_first != nullptr) {
        _collision_object_to_collider_entity[_selection_first]
            ->getSection(0)->getMaterial()->setDiffuse(Ogre::ColourValue(1, 1, 1));
        _collision_object_to_collider_entity[_selection_first]
            ->getSection(0)->getMaterial()->setSelfIllumination(Ogre::ColourValue(0, 0, 0));
    }

    if(_selection_second != nullptr) {
        _collision_object_to_collider_entity[_selection_second]
            ->getSection(0)->getMaterial()->setDiffuse(Ogre::ColourValue(1, 1, 1));
        _collision_object_to_collider_entity[_selection_second]
            ->getSection(0)->getMaterial()->setSelfIllumination(Ogre::ColourValue(0, 0, 0));
    }

    // Set new selection
    _selection_first = collider1;
    _selection_second = collider2;

    // Set color of new selection to red
    _collision_object_to_collider_entity[_selection_first]
        ->getSection(0)->getMaterial()->setDiffuse(Ogre::ColourValue(1, 0, 0));
    _collision_object_to_collider_entity[_selection_first]
        ->getSection(0)->getMaterial()->setSelfIllumination(Ogre::ColourValue(1, 0, 0));

    _collision_object_to_collider_entity[_selection_second]
        ->getSection(0)->getMaterial()->setDiffuse(Ogre::ColourValue(1, 0, 0));
    _collision_object_to_collider_entity[_selection_second]
        ->getSection(0)->getMaterial()->setSelfIllumination(Ogre::ColourValue(1, 0, 0));
}