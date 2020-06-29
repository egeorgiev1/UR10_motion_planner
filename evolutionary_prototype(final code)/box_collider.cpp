#include <string>

#include "box_collider.h"

using namespace std;

fcl::CollisionObject<real_t>* BoxCollider::_create_collision_object(
    const std::vector<fcl::Vector3d>& vertices,
    const std::vector<fcl::Triangle>& triangles
) {
    // Create and populate bounding volume hierarchy of the mesh data
    typedef fcl::BVHModel<fcl::OBBRSSd> bvh_model_t; // TODO: Change to fcl::OBBRSS<real_t>
    shared_ptr<bvh_model_t> model = make_shared<bvh_model_t>();
    
    // Add mesh data to BVH
    model->beginModel();
    model->addSubModel(vertices, triangles);
    model->endModel();

    // Create a collision object from the BVH
    //fcl::CollisionObject<real_t>* collider = new fcl::CollisionObject<real_t>{ model };
    fcl::CollisionObject<real_t>* collider = new fcl::CollisionObject<real_t>{
        make_shared<fcl::Boxd>(
            // TODO: REWORK THIS!!!
            vertices[1](0)*2,
            vertices[1](1)*2,
            vertices[1](2)*2
        )
    };

    return collider;
}

// DEPRECATED(USE PT_CUBE ENTITY INSTEAD, BECAUSE OF THE PROPER NORMALS)
Ogre::ManualObject* BoxCollider::_create_manual_object(
    const std::vector<fcl::Vector3d>& vertices,
    const std::vector<fcl::Triangle>& triangles
) {
    // Create/reuse material
    const std::string material_name = "collider_material"s;

    Ogre::MaterialPtr collider_material =
        Ogre::MaterialManager::getSingleton().getByName(material_name);
    
    if(not collider_material) {
        collider_material =
            Ogre::MaterialManager::getSingleton().create(
                material_name,
                Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME
            )
        ; 

        // Create wireframe material for this mesh(TODO: CHANGE FROM WIREFRAME TO SOLID MATERIAL?!)
        collider_material->setDiffuse(Ogre::ColourValue(1, 1, 1));
        //collider_material->setSelfIllumination(Ogre::ColourValue(1, 1, 1));
        //collider_material->getTechnique(0)->getPass(0)->setPolygonMode(Ogre::PM_WIREFRAME);
        collider_material->setCullingMode(Ogre::CullingMode::CULL_NONE); // Two-sided rendering
    }
    
    // Create ManualObject
    Ogre::ManualObject* manual_object = _scene_manager->createManualObject(_uuid);

    // Start setting up the mesh vertices and triangles
    manual_object->begin(
        material_name,
        Ogre::RenderOperation::OT_TRIANGLE_LIST
    );

    // Go over all verticies and add them in order to the ManualObject's buffer
    for(size_t i = 0; i < vertices.size(); i++) {
        manual_object->position(
            vertices[i](0),
            vertices[i](1),
            vertices[i](2)
        );
    }

    // Go over all triangles
    for(size_t i = 0; i < triangles.size(); i++) {
        manual_object->triangle(
            triangles[i][0],
            triangles[i][1],
            triangles[i][2]
        );
    }

    manual_object->end();

    return manual_object;
}

Ogre::SceneNode* BoxCollider::_create_scene_node(Ogre::ManualObject* manual_object)
{
    Ogre::SceneNode* node = _scene_manager->getRootSceneNode()->createChildSceneNode(_uuid);

    // Create/reuse material
    const std::string material_name = "collider_material"s;

    Ogre::MaterialPtr collider_material =
        Ogre::MaterialManager::getSingleton().getByName(material_name);
    
    if(not collider_material) {
        collider_material =
            Ogre::MaterialManager::getSingleton().create(
                material_name,
                Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME
            )
        ; 

        // Create wireframe material for this mesh(TODO: CHANGE FROM WIREFRAME TO SOLID MATERIAL?!)
        collider_material->setDiffuse(Ogre::ColourValue(1, 1, 1));
        //collider_material->setSelfIllumination(Ogre::ColourValue(1, 1, 1));
        //collider_material->getTechnique(0)->getPass(0)->setPolygonMode(Ogre::PM_WIREFRAME);
        collider_material->setCullingMode(Ogre::CullingMode::CULL_NONE); // Two-sided rendering
    }

    Ogre::Entity* box_entity = _scene_manager->createEntity(Ogre::SceneManager::PT_CUBE);
    box_entity->setMaterial(collider_material);
    // node->attachObject(manual_object);
    node->attachObject(box_entity);

    // Detach from parent
    node->getParent()->removeChild(node);

    return node;
}

BoxCollider::BoxCollider(
    std::string uuid,
    Ogre::SceneManager* scene_manager,
    fcl::BroadPhaseCollisionManager<real_t>* scene_collision_manager,
    std::array<real_t, 3> size, // Maybe redo this with scale and apply scale on fcl and ogre3d objects???
    std::array<real_t, 3> position,
    std::array<real_t, 4> orientation // quaternion
) :
    _uuid(uuid),
    _scene_manager(scene_manager),
    _scene_collision_manager(scene_collision_manager)
{
    std::vector<fcl::Vector3d> vertices {{
        // Front side vertices
        fcl::Vector3d(-size[0]/2,  size[1]/2,  size[2]/2),
        fcl::Vector3d( size[0]/2,  size[1]/2,  size[2]/2),
        fcl::Vector3d( size[0]/2, -size[1]/2,  size[2]/2),
        fcl::Vector3d(-size[0]/2, -size[1]/2,  size[2]/2),

        // Back side vertices
        fcl::Vector3d(-size[0]/2,  size[1]/2,  -size[2]/2),
        fcl::Vector3d( size[0]/2,  size[1]/2,  -size[2]/2),
        fcl::Vector3d( size[0]/2, -size[1]/2,  -size[2]/2),
        fcl::Vector3d(-size[0]/2, -size[1]/2,  -size[2]/2)
    }};
    
    std::vector<fcl::Triangle> triangles {{
        // Front side
        fcl::Triangle {0, 1, 2},
        fcl::Triangle {2, 3, 0},

        // Back side
        fcl::Triangle {4, 5, 6},
        fcl::Triangle {6, 7, 4},

        // Left side
        fcl::Triangle {4, 0, 3},
        fcl::Triangle {3, 7, 4},

        // Top side
        fcl::Triangle {4, 5, 1},
        fcl::Triangle {1, 0, 4},

        // Right side
        fcl::Triangle {1, 5, 6},
        fcl::Triangle {6, 2, 1},

        // Bottom side
        fcl::Triangle {3, 2, 6},
        fcl::Triangle {6, 7, 3}
    }};

    Ogre::SceneNode* scene_node = _create_scene_node(
        _create_manual_object(vertices, triangles)
    );

    // Apply translation and rotation
    // scene_node->setOrientation(
    //     Ogre::Quaternion(
    //         orientation[0],
    //         orientation[1],
    //         orientation[2],
    //         orientation[3]
    //     )
    // );

    // scene_node->setPosition(
    //     position[0],
    //     position[1],
    //     position[2]
    // );

    _scene_manager->getRootSceneNode()->addChild(
        scene_node
    );

    // Make the collision object
    fcl::CollisionObject<real_t>* collision_object =
        _create_collision_object(vertices, triangles);

    // Apply translation and rotation
    fcl::Transform3d transform =
        // fcl::Quaterniond(
        //     1, 0, 0, 0
        // ) *
        fcl::Translation3d(
            position[0],
            position[1],
            position[2]
        ) *
        fcl::Quaterniond(
            orientation[0],
            orientation[1],
            orientation[2],
            orientation[3]
        ) 
    ;
    
    collision_object->setTransform(
        transform
    );

    // Set scene node transform
    fcl::Vector3d transform_position = transform.translation();
    fcl::Matrix3d rotation_matrix = transform.rotation();
    fcl::Quaterniond rotation = fcl::Quaterniond(rotation_matrix);

    scene_node->resetToInitialState();

    // Apply rotation first
    scene_node->rotate(
        Ogre::Quaternion(
            rotation.w(),
            rotation.x(),
            rotation.y(),
            rotation.z()
        )
    );
    
    // Apply translation
    scene_node->setPosition(
        Ogre::Vector3(
            transform_position[0],
            transform_position[1],
            transform_position[2]
        )
    );

    // Apply corrective scaling
    scene_node->setScale(
        Ogre::Vector3(
            0.01*size[0],
            0.01*size[1],
            0.01*size[2]
        )
    );

    // Mandatory, because NaiveCollisionManager won't work properly without
    // properly computed AABB
    collision_object->computeAABB();

    // Register to the scene collision manager
    _scene_collision_manager->registerObject(
        collision_object
    );
}
