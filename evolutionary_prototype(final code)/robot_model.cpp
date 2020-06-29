#include "robot_model.h"

#include <string>
#include <vector>

#include "test_fcl_utility.h"

using namespace std;

RobotModel::CollisionObjectDescriptor RobotModel::_createCollisionObject(const string& object_name_param) {
    // HACK: TO BE FIXED!!!
    string object_name;
    if(object_name_param == "wrist1_"s) {
        object_name = "wrist1";
    } else if(object_name_param == "wrist2_"s) {
        object_name = "wrist2";
    } else {
        object_name = object_name_param;
    }

    // SAS FOR LOOP DA GO NAPRAVIA I SPISAK SAS IMENATA NA SAMITE MESHES!!!
    // set mesh triangles and vertice indices
    std::vector<fcl::Vector3d> vertices;
    std::vector<fcl::Triangle> triangles;
    
    // code to set the vertices and triangles
    string object_path = "../models/collision/"s + object_name + ".obj"s;
    fcl::test::loadOBJFile<double>(object_path.c_str(), vertices, triangles);

    // BVHModel is a template class for mesh geometry, for default OBBRSS template is used
    typedef fcl::BVHModel<fcl::OBBRSSd> Model; // TODO: da go promenia na fcl::OBBRSS<real_t>
    shared_ptr<Model> model = make_shared<Model>();
    
    // add the mesh data into the BVHModel structure
    model->beginModel();
    model->addSubModel(vertices, triangles);
    model->endModel();

    // Combine them together
    fcl::CollisionObject<double>* collider = new fcl::CollisionObject<double>(model);

    // Ogre3D mesh generation test
    Ogre::ManualObject* man = _scene_manager->createManualObject(object_name + "_collider"s);

    Ogre::MaterialPtr wireframeMat = Ogre::MaterialManager::getSingleton().create(object_name + "_collider_material"s, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
    wireframeMat->setDiffuse(Ogre::ColourValue(1, 1, 1));
    wireframeMat->setSelfIllumination(Ogre::ColourValue(1, 1, 1));
    wireframeMat->getTechnique(0)->getPass(0)->setPolygonMode(Ogre::PM_WIREFRAME);
    wireframeMat->setCullingMode(Ogre::CullingMode::CULL_NONE); // Two-sided rendering

    man->begin(object_name + "_collider_material"s, Ogre::RenderOperation::OT_TRIANGLE_LIST);

    // Go over all verticies and add them in order to ManualObject's buffer
    for(size_t i = 0; i < vertices.size(); i++) {
        man->position(
            vertices[i](0),
            vertices[i](1),
            vertices[i](2)
        );
    }

    // Go over all triangles
    for(size_t i = 0; i < triangles.size(); i++) {
        man->triangle(
            triangles[i][0],
            triangles[i][1],
            triangles[i][2]
        );
    }

    man->end();

    return {
        man,
        collider
    };
}

// xyz position and rpy orientation
Ogre::SceneNode* RobotModel::_createBase(
) {
    Ogre::SceneNode* baseNode =
        _scene_manager->getRootSceneNode()->createChildSceneNode("base"s);
    Ogre::Entity* ent = _scene_manager->createEntity("base.mesh");
    baseNode->attachObject(ent);

    CollisionObjectDescriptor collider_descriptor = _createCollisionObject("base"s);
    // _collision_objects.push_back(collider_descriptor.collider); // Is not one of the moving parts!!!
    _collision_manager->registerObject(collider_descriptor.collider);

    baseNode->attachObject(collider_descriptor.collider_entity);

    _collision_selection_manager->add(collider_descriptor.collider, collider_descriptor.collider_entity);

    _collision_entities.push_back(collider_descriptor.collider_entity);

    // TODO: Set default transform??? or let it be set in the joint angle methods???

    return baseNode;
}

Ogre::SceneNode* RobotModel::_createJoint(
    string mesh_name,
    size_t mesh_count,
    size_t joint_index
) {
    Ogre::SceneNode* dh_frame =
        _scene_manager->getRootSceneNode()->createChildSceneNode( mesh_name + "_dh_frame"s );

    _joint_nodes[joint_index] = dh_frame;

    // Geometry entities
    Ogre::SceneNode* geometry_frame = dh_frame->createChildSceneNode( mesh_name + "_geometry_frame");

    // Corner case naming of wrist3(TODO: FIX THE NAME!!!)
    if(mesh_name == "wrist3") {
        Ogre::Entity* subMesh = _scene_manager->createEntity(mesh_name + ".mesh");
        Ogre::SceneNode* subMeshNode = geometry_frame->createChildSceneNode( mesh_name );
        subMeshNode->attachObject(subMesh);
    } else {
        for(size_t i = 0; i < mesh_count; i++) {
            Ogre::Entity* subMesh = _scene_manager->createEntity(mesh_name + to_string(i+1) + ".mesh");
            Ogre::SceneNode* subMeshNode = geometry_frame->createChildSceneNode( mesh_name + to_string(i+1) );
            subMeshNode->attachObject(subMesh);
        }
    }

    // Create and register collider
    CollisionObjectDescriptor collider_descriptor = _createCollisionObject(mesh_name);
    _collision_objects.push_back(collider_descriptor.collider);
    _collision_manager->registerObject(collider_descriptor.collider);
    _collision_selection_manager->add(collider_descriptor.collider, collider_descriptor.collider_entity);
    _collision_entities.push_back(collider_descriptor.collider_entity);

    geometry_frame->attachObject(collider_descriptor.collider_entity);

    return geometry_frame;
}

// TODO: Toggle visibility from a grouping scene node from the outside!!!
void RobotModel::toggle_collider_visibility() {
    colliders_visible = !colliders_visible;

    for(Ogre::ManualObject* obj: _collision_entities) {
        obj->setVisible(colliders_visible);
    }
}

void RobotModel::toggle_visual_mesh_visibility() {
    visual_mesh_visible = !visual_mesh_visible;

    for(Ogre::SceneNode* obj: _joint_nodes) {
        obj->setVisible(visual_mesh_visible);
    }
}

RobotModel::RobotModel(
    Ogre::SceneManager* scene_manager,
    fcl::BroadPhaseCollisionManager<double>* collision_manager,
    CollisionSelectionManager* collision_selection_manager,
    ForwardKinematics* fk
) :
    _scene_manager(scene_manager),
    _collision_manager(collision_manager),
    _joint_angles{-0.54, 0.71, -1.98, -2.04, 1.01, -0.26}, // TODO: MUST BE INJECTED FROM THE OUTSIDE
    _joint_nodes(6, nullptr),
    _collision_selection_manager(collision_selection_manager),
    _fk(fk)
{
    Ogre::SceneNode* baseNode     = _createBase();
    Ogre::SceneNode* shoulderNode = _createJoint("shoulder", 3, 0);
    Ogre::SceneNode* upperarmNode = _createJoint("upperarm", 4, 1);
    Ogre::SceneNode* forearmNode  = _createJoint("forearm", 5, 2);
    Ogre::SceneNode* wrist1Node   = _createJoint("wrist1_", 3, 3);
    Ogre::SceneNode* wrist2Node   = _createJoint("wrist2_", 3, 4);
    Ogre::SceneNode* wrist3Node   = _createJoint("wrist3", 1, 5);

    // Apply initial angles
    set_angles(_joint_angles);
}

vector<real_t> RobotModel::get_angles() const {
    return _joint_angles;
}

// TODO: WRITE MY OWN VERSION: https://forums.ogre3d.org/viewtopic.php?t=53647
void RobotModel::_DestroyAllAttachedMovableObjects(Ogre::SceneNode* i_pSceneNode)
{
    using namespace Ogre;
    if ( !i_pSceneNode )
    {
        assert(false);
        return;
    }

    // Destroy all the attached objects
    SceneNode::ObjectIterator itObject = i_pSceneNode->getAttachedObjectIterator();

    while ( itObject.hasMoreElements() )
    {
        MovableObject* pObject = static_cast<MovableObject*>(itObject.getNext());
        i_pSceneNode->getCreator()->destroyMovableObject( pObject );
    }

    // Recurse to child SceneNodes
    SceneNode::ChildNodeIterator itChild = i_pSceneNode->getChildIterator();

    while ( itChild.hasMoreElements() )
    {
        SceneNode* pChildNode = static_cast<SceneNode*>(itChild.getNext());
        _DestroyAllAttachedMovableObjects( pChildNode );
    }
}

// NOTE: Do for all joints, because one angle migth affect more than one joint! Optimization
// for reducing the updates could be done, but most of the time with IK we'll update all angles
// at once
void RobotModel::update_collider_transforms() {
    for(size_t i = 0; i < 6; i++) {

        _collision_objects[i]->setTransform(_fk->get_transform(i));

        /*
        Vav tozi testing example se vizda, che callvat computeAABB vseki put sled promeniane na samia CollisionObject transform:
        https://github.com/flexible-collision-library/fcl/blob/9dba579158109c0164bfe0e8b4a75c16bfc788f6/test/test_fcl_broadphase_collision_1.cpp

        Tuka obsazdat fakta, che moze Broadphase sas AABB da ti dava korekten otgovori dori da ne vikash computeAABB, no moze da e mnogo po-baven:
        https://github.com/flexible-collision-library/fcl/issues/40
        */
        _collision_objects[i]->computeAABB(); // Manually triabva da se callva. Pri search iz samia source code computeAABB kato cialo se callva manually vav tests ili vav samia CollisionObject constructor

        // cout << endl;
        // cout << transform.matrix() << endl << endl;
        // cout << _fk->get_transform(i).matrix() << endl << "END" << endl;
    }

    // Setup the managers, which is related with initializing the broadphase acceleration structure according to objects input
    _collision_manager->setup(); // TODO: CALL ONLY ONCE OR AFTER EVERY COLLISION MANAGER UPDATE??? Ot source code izliza, che ima check, koito gleda dali veche e setupnato ili ne

    // Update the broadphase collision manager data structure
    _collision_manager->update(_collision_objects); // MORE OPTIMAL, BUT TEST IT FURTHER SOME OTHER TIME!!!

    // Perform collision detection(TODO: OPTIMIZE)
    fcl::test::CollisionData<double> collision_data; // MANIPULATE .request FOR MORE CONTACT POINTS!!!
    // fcl::test::DistanceData<double> distance_data;

    collision_data.request.enable_contact = true; // kogato e false dava nepravilni results?? taka li shte e i sas collision samo mezdu 2 objects???
    const intmax_t NUMBER_OF_CONTACT_POINTS = 5000;
    collision_data.request.num_max_contacts = NUMBER_OF_CONTACT_POINTS;
    
    //_collision_manager->collide(_collision_manager, &collision_data, fcl::test::defaultCollisionFunction<double>);
    _collision_manager->collide(&collision_data, fcl::test::defaultCollisionFunction<double>);
    
    // fcl::CollisionRequest<double> request;
    // request.enable_contact = true;
    // request.num_max_contacts = 20;
    // // result will be returned via the collision result structure
    // fcl::CollisionResult<double> result;

    // const size_t selection1 = 1;
    // const size_t selection2 = 4;
    // fcl::collide(_collision_objects[selection1], _collision_objects[selection2], request, result);
    // _collision_selection_manager->select_pair(_collision_objects[selection1], _collision_objects[selection2]);
    // cout << "COLLISION NUMBER OF CONTACTS: " << result.numContacts() << endl;
    
    // static size_t collision_instance = 0;
    // for(size_t i = 0; i < result.numContacts(); i++) {
    //     Ogre::Entity* sphereEntity = _scene_manager->createEntity(Ogre::SceneManager::PT_SPHERE);
    //     //contactPointsMaterial->setCullingMode(Ogre::CullingMode::CULL_NONE); // Two-sided rendering

    //     Ogre::MaterialPtr contactPointsMaterial = Ogre::MaterialManager::getSingleton().getByName("contact_points_material"s);
    //     sphereEntity->setMaterial(contactPointsMaterial);
        
    //     Ogre::SceneNode* contactPointNode = _scene_manager->getRootSceneNode()->createChildSceneNode("contact_point_node"s + to_string(collision_instance) + "_" + to_string(i));
    //     contactPointNode->setScale(Ogre::Vector3(0.00005, 0.00005, 0.00005));
    //     contactPointNode->setPosition(Ogre::Vector3(
    //         result.getContact(i).pos.x(),
    //         result.getContact(i).pos.y(),
    //         result.getContact(i).pos.z()
    //     ));
        
    //     contactPointNode->attachObject(sphereEntity);
    // }
    // collision_instance++;

    // Visualize contact points
    static size_t collision_instance = 0;
    static Ogre::SceneNode* contact_points[NUMBER_OF_CONTACT_POINTS];

    if(collision_instance == 0) {
        for(intmax_t i = 0; i < NUMBER_OF_CONTACT_POINTS; i++) {
            contact_points[i] = nullptr;
        }
    }

    // Moze vmesto destroying da se reusevat??? da imam pool ot spheres!!!, a ako imam tvurde mnogo spheres???
    // togava ostanalite da ne gi rendervam pri takova polozenie, da ima maximum limit na spheres za samite contact
    // points!!!
    for(size_t i = 0; i < NUMBER_OF_CONTACT_POINTS; i++) {
        if (contact_points[i] != nullptr) {
            _DestroyAllAttachedMovableObjects(contact_points[i]);
            contact_points[i]->removeAndDestroyAllChildren();
            _scene_manager->destroySceneNode(contact_points[i]);

            contact_points[i] = nullptr;
        }
    }

    for(size_t i = 0; i < collision_data.result.numContacts(); i++) {
        Ogre::Entity* sphereEntity = _scene_manager->createEntity(Ogre::SceneManager::PT_SPHERE);
        Ogre::MaterialPtr contactPointsMaterial = Ogre::MaterialManager::getSingleton().getByName("contact_points_material"s);
        sphereEntity->setMaterial(contactPointsMaterial);
        
        Ogre::SceneNode* contactPointNode = _scene_manager->getRootSceneNode()->createChildSceneNode("contact_point_node"s + to_string(collision_instance) + "_" + to_string(i));
        contact_points[i] = contactPointNode;
        contactPointNode->setScale(Ogre::Vector3(0.0002, 0.0002, 0.0002));
        contactPointNode->setPosition(Ogre::Vector3(
            collision_data.result.getContact(i).pos.x(),
            collision_data.result.getContact(i).pos.y(),
            collision_data.result.getContact(i).pos.z()
        ));
        
        contactPointNode->attachObject(sphereEntity);
    }
    collision_instance++;

    cout << "COLLISION MANAGER NUMBER OF CONTACTS: " << collision_data.result.numContacts() << endl;
}

void RobotModel::set_angle(size_t joint_index, real_t angle)
{
    assert(joint_index < _joint_angles.size());
    // assert(/* angle in the range of [-PI; PI) */);

    _joint_angles[joint_index] = angle;

    // Update forward kinematics
    _fk->set_angle(joint_index, angle);

    // Get DH frame and decompose into position, roll, pitch and yaw
    // TODO: Implement using the Eigen namespace instead of using the fcl namespace
    fcl::Transform3d dh_frame = _fk->get_transform(joint_index);

    fcl::Vector3d position = dh_frame.translation();
    fcl::Matrix3d rotation_matrix = dh_frame.rotation();
    fcl::Quaterniond rotation = fcl::Quaterniond(rotation_matrix);

    //fcl::Vector3d euler_angles = rotation_matrix.eulerAngles(0, 1, 2);
    
    _joint_nodes[joint_index]->resetToInitialState();

    // Apply rotation first (cleaner using quaternions)
    _joint_nodes[joint_index]->rotate(
        Ogre::Quaternion(
            rotation.w(),
            rotation.x(),
            rotation.y(),
            rotation.z()
        )
    );
    
    //_joint_nodes[joint_index]->pitch(Ogre::Radian(euler_angles[0])); // X axis : https://ogrecave.github.io/ogre/api/1.11/tiki-download_file_rotation_axis.png
    //_joint_nodes[joint_index]->yaw(Ogre::Radian(euler_angles[1])); // Y axis : https://ogrecave.github.io/ogre/api/1.11/tiki-download_file_rotation_axis.png
    //_joint_nodes[joint_index]->roll(Ogre::Radian(euler_angles[2])); // Z axis : https://ogrecave.github.io/ogre/api/1.11/tiki-download_file_rotation_axis.png

    // Apply translation
    _joint_nodes[joint_index]->setPosition(
        Ogre::Vector3(
            position[0],
            position[1],
            position[2]
        )
    );
}

void RobotModel::rotate_joint(size_t joint_index, real_t angle)
{
    // You want to call a private method instead, but that works for now
    set_angle(
        joint_index,
        _joint_angles[joint_index] + angle
    );

    // Update all links after it
    for(size_t i = joint_index + 1; i < 6; i++) {
        set_angle(
            i,
            _joint_angles[i]
        );
    }
    

    update_collider_transforms();
}
