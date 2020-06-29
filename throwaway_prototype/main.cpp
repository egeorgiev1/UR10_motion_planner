// #define TINYOBJLOADER_IMPLEMENTATION // define this in only *one* .cc
// #include "tiny_obj_loader.h"

#include <map>
#include <memory>
#include <algorithm>
#include <Eigen/StdVector>

#include "Ogre.h"
#include "OgreApplicationContext.h"
#include "OgreCameraMan.h"
#include "OgreInput.h"

#include "fcl/fcl.h"
#include "test_fcl_utility.h"
#include "fcl/broadphase/broadphase_collision_manager.h"
#include "fcl/broadphase/broadphase_dynamic_AABB_tree.h"
#include "fcl/broadphase/broadphase_bruteforce.h"

#include <kdl/chain.hpp>
// #include <kdl/chainfksolver.hpp>
// #include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/frames_io.hpp>

#include <iostream>
using namespace std;

using namespace KDL;

typedef double real_t;
enum class Axis { X, Y, Z };

class AxisHelperFactory {
private:
    Ogre::SceneManager* _scene_manager {nullptr};

    Ogre::ManualObject* _get_axis_object(string name, array<real_t, 3> color, array<real_t, 3> direction) {
        
        // Create world axes and attach them to the scene
        Ogre::ManualObject* axis;

        if(!_scene_manager->hasManualObject(name)) {

            axis = _scene_manager->createManualObject(name); 

            Ogre::MaterialPtr axis_material = Ogre::MaterialManager::getSingleton().create(name + "_material"s, "General"s); 
            axis_material->setReceiveShadows(false); 
            axis_material->getTechnique(0)->setLightingEnabled(true); 
            axis_material->getTechnique(0)->getPass(0)->setDiffuse(color[0], color[1], color[2], 0); 
            axis_material->getTechnique(0)->getPass(0)->setAmbient(color[0], color[1], color[2]); 
            axis_material->getTechnique(0)->getPass(0)->setSelfIllumination(color[0], color[1], color[2]); 

            axis->begin(name + "_material"s, Ogre::RenderOperation::OT_LINE_LIST);
            axis->position(0, 0, 0);
            axis->position(direction[0], direction[1], direction[2]);
            axis->end(); 

        } else {
            axis = _scene_manager->getManualObject(name); 
        }

        return axis;
    }

public:
    AxisHelperFactory(Ogre::SceneManager* scene_manager) :
        _scene_manager(scene_manager)
    {}

    Ogre::SceneNode* get_axes_node(string name) {
        
        // Return detached scene node if it already exists
        Ogre::SceneNode* axes_group {nullptr};
        
        if(_scene_manager->hasSceneNode(name))
        {
            // Get node
            axes_group = _scene_manager->getSceneNode(name);

            // Detach node
            axes_group->getParent()->removeChild(axes_group);
        }
        else {
            // Create axes group
            axes_group =
                _scene_manager->getRootSceneNode()->createChildSceneNode(name);

            // Create X axis
            Ogre::ManualObject* x_axis_object =
                _get_axis_object(name + "_x_axis", {1, 0, 0}, {1, 0, 0});
            
            Ogre::SceneNode* x_axis_node =
                axes_group->createChildSceneNode(name + "_x_axis_node"s); 
            
            x_axis_node->attachObject(x_axis_object);

            // Create Y axis
            Ogre::ManualObject* y_axis_object =
                _get_axis_object(name + "_y_axis", {0, 1, 0}, {0, 1, 0});
            Ogre::SceneNode* y_axis_node =
                axes_group->createChildSceneNode(name + "_y_axis_node"s); 
            
            y_axis_node->attachObject(y_axis_object);

            // Create Z axis
            Ogre::ManualObject* z_axis_object =
                _get_axis_object(name + "_z_axis", {0, 0, 1}, {0, 0, 1});
            Ogre::SceneNode* z_axis_node =
                axes_group->createChildSceneNode(name + "_z_axis_node"s); 
            
            z_axis_node->attachObject(z_axis_object);

            // Detach node
            axes_group->getParent()->removeChild(axes_group);
        }

        return axes_group;
    }
};

// Objects that implement OgreBites::InputListener can
// register their input handlers to an InputManager object.
// This must be done only by controller objects!
class InputManager : public OgreBites::InputListener {
private:
    vector<OgreBites::InputListener*> _listeners;
public:
    InputManager() {}

    void add_listener(OgreBites::InputListener* listener) {
        _listeners.push_back(listener);
    }

    bool keyPressed(const OgreBites::KeyboardEvent& evt) override {
        for(OgreBites::InputListener *const listener: _listeners) {
            listener->keyPressed(evt);
        }
    }

    void frameRendered (const Ogre::FrameEvent& evt) override {
        for(OgreBites::InputListener *const listener: _listeners) {
            listener->frameRendered(evt);
        }
    }

    bool keyReleased (const OgreBites::KeyboardEvent& evt) override {
        for(OgreBites::InputListener *const listener: _listeners) {
            listener->keyReleased(evt);
        }
    }

    bool mouseMoved (const OgreBites::MouseMotionEvent& evt) override {
        for(OgreBites::InputListener *const listener: _listeners) {
            listener->mouseMoved(evt);
        }
    }

    bool mousePressed (const OgreBites::MouseButtonEvent& evt) override {
        for(OgreBites::InputListener *const listener: _listeners) {
            listener->mousePressed(evt);
        }
    }

    bool mouseReleased (const OgreBites::MouseButtonEvent& evt) override {
        for(OgreBites::InputListener *const listener: _listeners) {
            listener->mouseReleased(evt);
        }
    }

    bool mouseWheelRolled (const OgreBites::MouseWheelEvent& evt) override {
        for(OgreBites::InputListener *const listener: _listeners) {
            listener->mouseWheelRolled(evt);
        }
    }

    bool touchMoved (const OgreBites::TouchFingerEvent& evt) override {
        for(OgreBites::InputListener *const listener: _listeners) {
            listener->touchMoved(evt);
        }
    }

    bool touchPressed (const OgreBites::TouchFingerEvent& evt) override {
        for(OgreBites::InputListener *const listener: _listeners) {
            listener->touchPressed(evt);
        }
    }

    bool touchReleased (const OgreBites::TouchFingerEvent& evt) override {
        for(OgreBites::InputListener *const listener: _listeners) {
            listener->touchReleased(evt);
        }
    }
};

// End effector visualization
class EndEffectorView // Use an IEndEffectorView interface???
{
private:
    Ogre::SceneManager* _scene_manager {nullptr};
    AxisHelperFactory*_axis_helper_factory {nullptr};
    
    Ogre::SceneNode* _end_effector {nullptr};
public:
    EndEffectorView(Ogre::SceneManager* scene_manager, AxisHelperFactory* axis_helper_factory)
    :
        _scene_manager(scene_manager),
        _axis_helper_factory(axis_helper_factory)
    {
        // Create end effector group node
        _end_effector = _scene_manager->getRootSceneNode()->createChildSceneNode("end_effector");

        // Create red sphere entity
        Ogre::Entity* sphere_entity = _scene_manager->createEntity(Ogre::SceneManager::PT_SPHERE); // Use a weak_ptr??? Who will manage the deletion of this object???
        sphere_entity->getSubEntity(0)->getMaterial()->setDiffuse(Ogre::ColourValue(1, 0, 0));
        sphere_entity->getSubEntity(0)->getMaterial()->setAmbient(Ogre::ColourValue(1, 0, 0)); // Derive from diffuse???
        
        // Scale sphere node properly
        Ogre::SceneNode* sphere_node =
            _scene_manager->getRootSceneNode()->createChildSceneNode("sphere_node");

        sphere_node->setScale(Ogre::Vector3(0.0002, 0.0002, 0.0002));
        sphere_node->setPosition(Ogre::Vector3(0, 0, 0)); // TODO: Is this needed???

        sphere_node->attachObject(sphere_entity);
        sphere_node->getParentSceneNode()->removeChild(sphere_node);

        // Create local axes for visualizing end effector orientation
        Ogre::SceneNode* axes_helper_node = _axis_helper_factory->get_axes_node("end_effector_axes"s);
        axes_helper_node->setPosition(0.0001, 0, 0); // Improvement on flickering issue from interleaving with the grid. POSSIBLE SOLUTION: use cylinders instead of lines
        axes_helper_node->setScale(0.1, 0.1, 0.1);

        // Attach the sphere and the axes visualization to the end effector node
        _end_effector->addChild(sphere_node);
        _end_effector->addChild(axes_helper_node);
    }

    void set_position(real_t x, real_t y, real_t z) {
        _end_effector->setPosition( {x, y, z} ); // Will this work???

        // DEBUG
        cout << "END EFFECTOR POSITION SET:" << endl;
        cout << "x: " << x << endl;
        cout << "y: " << y << endl;
        cout << "z: " << z << endl;
        cout << endl;
    }

    // Refactor using custom radian literals and radian wrappers??? Don't use the Ogre3D ones. It's
    // an external library to which I don't want to couple the interface.
    void set_orientation(real_t yaw, real_t pitch, real_t roll) {
        _end_effector->resetOrientation();

        // TODO: TEMPORARILY FIXING THE AXES!!! UNDO THIS LATER!!!
        _end_effector->yaw( Ogre::Radian{ roll } );
        _end_effector->pitch( Ogre::Radian{ -yaw } );
        _end_effector->roll( Ogre::Radian{ pitch } );

        // DEBUG
        cout << "END EFFECTOR ORIENTATION SET:" << endl;
        cout << "Yaw: " << yaw << endl;
        cout << "Pitch: " << pitch << endl;
        cout << "Roll: " << roll << endl;
        cout << endl;
    }
};

class InverseKinematics {
private:
    array<real_t, 3> _target_position {{0, 0, 0}};
	array<real_t, 3> _target_orientation {{0, 0, 0}};
	array<real_t, 6> _current_angles {{0, 0, 0, 0, 0, 0}};
	string _last_error;

    // Used for initializing ik_solver. If not done like this ik_solver would need to be a pointer.
    static KDL::Chain _joint_frames() {
        KDL::Chain _joint_frames;

        // Setup joint frames of the UR10 using DH parameters
        _joint_frames.addSegment(Segment(Joint(Joint::RotZ), Frame::DH(0,        M_PI_2,  0.1273,   0)));
        _joint_frames.addSegment(Segment(Joint(Joint::RotZ), Frame::DH(-0.612,   0,       0,        0)));
        _joint_frames.addSegment(Segment(Joint(Joint::RotZ), Frame::DH(-0.5723,  0,       0,        0)));
        _joint_frames.addSegment(Segment(Joint(Joint::RotZ), Frame::DH(0,        M_PI_2,  0.163941, 0)));
        _joint_frames.addSegment(Segment(Joint(Joint::RotZ), Frame::DH(0,       -M_PI_2,  0.1157,   0)));
        _joint_frames.addSegment(Segment(Joint(Joint::RotZ), Frame::DH(0,        0,       0.0922,   0)));

        return _joint_frames;
    }


    KDL::Chain _joints_chain;
    
    // TODO: Why LMA over Newtonian? How it handles singularities by going around
    // them??? Da vida dali taka behaveva i da vida da dam reference za tozi claim
    KDL::ChainIkSolverPos_LMA* _ik_solver {new ChainIkSolverPos_LMA{_joint_frames()}};

public:
    InverseKinematics()
    :
        _joints_chain{_joint_frames()}
    {
        _ik_solver = new ChainIkSolverPos_LMA{_joints_chain};
    }

	void set_target_position(const array<real_t, 3>& position) {
        _target_position = position;
    }
	
    void set_target_orientation(const array<real_t, 3>& orientation) { // In RPY format
        _target_orientation = orientation;
    }
	
    void set_current_angles(const array<real_t, 6>& current_angles) {
        _current_angles = current_angles;
    }

    // If there's an error it will return -1, otherwise it will return 0
	intmax_t solve(array<real_t, 6>& output_angles) {
        // Create input and output arrays of angles
        JntArray initial_angles(_current_angles.size());
        JntArray resulting_angles(_current_angles.size());
        
        // Initialize input array of angles
        for(size_t i = 0; i < _current_angles.size(); i++) {
            initial_angles(i) = _current_angles[i];
        }
        
        // Create destination frame
        Frame destination_frame = Frame(
            KDL::Rotation::RPY(_target_orientation[0], _target_orientation[1], _target_orientation[2]),
            KDL::Vector(_target_position[0], _target_position[1], _target_position[2])
        );

        // Run the solver
        int error_code = _ik_solver->CartToJnt(initial_angles, destination_frame, resulting_angles);

        // Set output angles
        for(size_t i = 0; i < _current_angles.size(); i++) {
            output_angles[i] = resulting_angles(i);
        }

        // Set error string
        if(error_code == KDL::SolverI::E_NOERROR) {
            _last_error = ""s;
        } else {
            _last_error = _ik_solver->strError(error_code);
        }

        // Return 0 if no error, otherwise return -1
        if(error_code == KDL::SolverI::E_NOERROR) {
            return 0;
        } else {
            return -1;
        }
    }
	
    string get_last_error() {
        return _last_error;
    }
};

class ForwardKinematics {
private:
    fcl::aligned_vector<fcl::Transform3d>* _local_reference_frame;
    vector<Axis> _rotation_axis;
    vector<bool> _reverse_rotation_direction;

    fcl::aligned_vector<fcl::Transform3d> _global_reference_frame;
    vector<bool> _invalidated; // So that _global_reference_frame is lazily evaluated without unnecessary matrix multiplications
    vector<real_t> _joint_angles;
public:
    ForwardKinematics(fcl::aligned_vector<fcl::Transform3d>* local_reference_frame, vector<Axis>& rotation_axis, vector<bool>& reverse_rotation_direction) // TODO: MUST check if all invariants are met and that the objects and pointers are valid!!!
    :
        _local_reference_frame(local_reference_frame),
        _global_reference_frame(6, fcl::Transform3d::Identity()),
        _rotation_axis(rotation_axis),
        _invalidated(6, true), // HARDCODED 6 DEGREES OF FREEDOM???
        _joint_angles(6, 0.0 /* 0.0_r */), // HARDCODED 6 DEGREES OF FREEDOM???
        _reverse_rotation_direction(reverse_rotation_direction)
    {
    }

    void set_angle(size_t joint_index, real_t angle) { // TODO: check invariants
        for(size_t i = joint_index; i < _invalidated.size(); i++) {
            _invalidated[i] = true;
        }

        _joint_angles[joint_index] = angle;
    }

    void rotate(size_t joint_index, real_t angle) {
        if(_reverse_rotation_direction[joint_index]) {
            angle = (-angle);
        }

        set_angle(joint_index, _joint_angles[joint_index] + angle);
    }

    // No problem with returning Eigen objects: https://eigen.tuxfamily.org/dox/group__TopicPassingByValue.html
    fcl::Transform3d get_transform(size_t joint_index) {
        if(_invalidated[joint_index]) {
            // Find first index that's not invalidated
            ssize_t valid_transform_index;
            for(valid_transform_index = joint_index; valid_transform_index >= 0; valid_transform_index--) {
                if(!_invalidated[valid_transform_index]) {
                    break;
                }
            }

            // Update transforms after the first valid index
            for(size_t i = valid_transform_index + 1; i <= joint_index; i++) {
                fcl::Vector3d axis_of_rotation;

                // HACK: Axes mapped to roll, pitch and yaw in Ogre3D: https://ogrecave.github.io/ogre/api/1.11/tiki-download_file_rotation_axis.png
                if(_rotation_axis[i] == Axis::X) {
                    axis_of_rotation = -fcl::Vector3d::UnitZ(); // WHY DOES THIS NEED TO BE INVERSED???
                } else if(_rotation_axis[i] == Axis::Y) {
                    axis_of_rotation = fcl::Vector3d::UnitY();
                } else if(_rotation_axis[i] == Axis::Z) {
                    axis_of_rotation = fcl::Vector3d::UnitX();
                } else {
                    // Error, but is it possible with enum class???
                    throw "NO VALID AXIS SPECIFIED";
                }
                
                if(i == 0) {
                    _global_reference_frame[i] = 
                        _local_reference_frame->at(i) *
                        fcl::AngleAxisd(_joint_angles[i], axis_of_rotation); // Why is the rotation last? Is the angle I'm using the angle for the joint after the current one???
                } else {
                    _global_reference_frame[i] = 
                        _global_reference_frame[i - 1] *
                        _local_reference_frame->at(i) *
                        fcl::AngleAxisd(_joint_angles[i], axis_of_rotation);
                }

                _invalidated[i] = false;
            }
        }

        return _global_reference_frame[joint_index];
    }
};

class CollisionSelectionManager { // pass this to every collision object creator!!!
private:
    // map from collisionobject pointer to scenenode pointer
    // currently selected object
    
    map<fcl::CollisionObject<double>*, Ogre::ManualObject*> _collision_object_to_collider_entity;
    fcl::CollisionObject<double>* _selection_first = nullptr;
    fcl::CollisionObject<double>* _selection_second = nullptr;
public:
    void add(fcl::CollisionObject<double>* collider, Ogre::ManualObject* collider_entity) {
        _collision_object_to_collider_entity[collider] = collider_entity;
    }

    // Select colliding pair of objects
    void select_pair(fcl::CollisionObject<double>* collider1, fcl::CollisionObject<double>* collider2) {
        // If collider is not in the data structures then throw an exception!!!
        if(_collision_object_to_collider_entity.find(collider1) == _collision_object_to_collider_entity.end()) {
            throw "collider1 not in CollisionSelectionManager!";
        }

        if(_collision_object_to_collider_entity.find(collider2) == _collision_object_to_collider_entity.end()) {
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
};

class RobotModel {
private:
	Ogre::SceneManager* _scene_manager;
	fcl::BroadPhaseCollisionManager<double>* _collision_manager;
    ForwardKinematics* _fk;

	vector<real_t> _joint_angles;
	vector<Ogre::SceneNode*> _joint_nodes; // who owns these SceneNode pointers? Can I make them shared_ptr instead of plain-old C pointers???
    vector<bool> _reverse_rotation_direction;
    vector<Axis> _axis_of_rotation;

    // In the order corresponding to _joint_angles for transform updates!!!
    CollisionSelectionManager* _collision_selection_manager;
    vector<fcl::CollisionObject<double>*> _collision_objects; // who owns these pointers? Can I make them shared_ptr?
    vector<Ogre::ManualObject*> _collision_entities; // used for toggling visibility!
    bool colliders_visible = true;
    bool visual_mesh_visible = true;

    struct CollisionObjectDescriptor {
        Ogre::ManualObject* collider_entity;
        fcl::CollisionObject<double>* collider;
    };

    CollisionObjectDescriptor _createCollisionObject(const string& object_name_param) {
        // HACK: TO BE FIXED!!!
        string object_name;
        if(object_name_param == "wrist1_"s) {
            object_name = "wrist1";
        } else if(object_name_param == "wrist2_"s) {
            object_name = "wrist2";
        } else {
            object_name = object_name_param;
        }

        // const size_t JOINTS_COUNT = 6;

        // vector<string> mesh_names {
        //     "base"s,
        //     "shoulder"s,
        //     "upperarm"s,
        //     "forearm"s,
        //     "wrist1"s,
        //     "wrist2"s,
        //     "wrist3"s
        // };

        // Attach the collision objects to SceneNodes

        // Create FCL collision objects

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

        // // R and T are the rotation matrix and translation vector
        
        // code for setting R and T
        // TODO: SET THEM TO SOMETHING DEFAULT, ZEROS???

        // transform is configured according to R and T
        
        // Placeholder transform, maybe even discard quaternions and translations???
        // fcl::Quaterniond R;
        // fcl::Translation3d T;
        // fcl::Transform3d pose = R*T;

        // Combine them together
        // fcl::CollisionObject<double>* obj = new fcl::CollisionObject<double>(model, pose);
        fcl::CollisionObject<double>* collider = new fcl::CollisionObject<double>(model);

        // Ogre3D mesh generation test
        Ogre::ManualObject* man = _scene_manager->createManualObject(object_name + "_collider"s);
        // man->begin("Examples/OgreLogo", Ogre::RenderOperation::OT_TRIANGLE_LIST);
        // man->begin("white_grid", Ogre::RenderOperation::OT_TRIANGLE_LIST); // TODO: make a separate material for each collision object so that we can change its color at runtime for indicating collision. How to get the query result properly from FCL???

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
            // man->normal(1, 1, 1);
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

        // Ogre::SceneNode* node = _scene_manager->getRootSceneNode()->createChildSceneNode();
        // node->attachObject(man);

        // EXAMPLE: Mark object as colliding
        // dynamic_cast<Ogre::ManualObject*>(node->getAttachedObject(0))->getSection(0)->getMaterial()->setDiffuse(Ogre::ColourValue(1, 0, 0));

        return {
            man,
            collider
        };

    }

    // xyz position and rpy orientation
    Ogre::SceneNode* _createBase(Ogre::SceneNode* parentNode, Ogre::Vector3 position, Ogre::Quaternion orientation) {
        Ogre::SceneNode* baseNode = parentNode->createChildSceneNode("base"s);
        baseNode->setPosition(position);
        baseNode->setOrientation(orientation);
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

    Ogre::SceneNode* _createJoint(
        Ogre::SceneNode* parentNode,
        Ogre::Vector3 position,
        real_t Rx,
        real_t Ry,
        real_t Rz,
        Axis axis_of_rotation,
        bool reverse_rotation_direction,
        string mesh_name,
        size_t mesh_count,
        size_t joint_index
    ) {
        // DH frame
        Ogre::Matrix3 orientation;
        orientation.FromEulerAnglesXYZ(
            Ogre::Radian(Rx), Ogre::Radian(Ry), Ogre::Radian(Rz)
        );

        Ogre::SceneNode* dh_frame = parentNode->createChildSceneNode( mesh_name + "_dh_frame"s );
        dh_frame->setPosition(position);
        dh_frame->setOrientation(Ogre::Quaternion(orientation));

        // Rotation frame
        Ogre::SceneNode* rotation_frame = dh_frame->createChildSceneNode( mesh_name + "_rotation_frame"s );
        _joint_nodes[joint_index] = rotation_frame;
        _reverse_rotation_direction[joint_index] = reverse_rotation_direction;
        _axis_of_rotation[joint_index] = axis_of_rotation;

        // Geometry entities
        Ogre::SceneNode* geometry_frame = rotation_frame->createChildSceneNode( mesh_name + "_geometry_frame");

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

public:
    void toggle_collider_visibility() {
        colliders_visible = !colliders_visible;

        for(Ogre::ManualObject* obj: _collision_entities) {
            obj->setVisible(colliders_visible);
        }
    }

    void toggle_visual_mesh_visibility() {
        visual_mesh_visible = !visual_mesh_visible;

        for(Ogre::SceneNode* obj: _joint_nodes) {
            obj->setVisible(visual_mesh_visible);
        }
    }

	RobotModel(
        Ogre::SceneManager* scene_manager,
        fcl::BroadPhaseCollisionManager<double>* collision_manager,
        CollisionSelectionManager* collision_selection_manager,
        ForwardKinematics* fk
    ) :
		_scene_manager(scene_manager),
		_collision_manager(collision_manager),
		_joint_angles{-0.54, 0.71, -1.98, -2.04, 1.01, -0.26},
		_joint_nodes(6, nullptr),
        _reverse_rotation_direction(6, false),
        _axis_of_rotation(6, Axis::Y),
        _collision_selection_manager(collision_selection_manager),
        _fk(fk)
	{
        // URDF coordinate system is a right-handed system, where Z is up, Y is left and X
        // is forward(directions relative to your right arm in the grip of a right-handed coordinate system)
        // To translate to Ogre3D's coordinate system:
        // XYZ Rx Ry Rz - ORIGINAL URDF COORDINATES
        // XZ-Y Rz Rx -Ry - rotation coordinates correct??? zashto Rz i Rx sa taka???

        /* Rotation Direction:

        DH params coordinate system:

        Rx - roll
        Ry - pitch
        Rz - yaw

        RPY - row, pitch, yaw

        with an XYZ coordinate system the positive direction of rotation is defined as the following:

        For an Rz rotation you rotate X towards Y
        For an Rx rotation you rotate Y towards Z
        For an Ry rotation you rotate Z towards X

        TRANSFORMED:

        RPY
        X Z -Y - formula to get the right coordinates

        R: Z -> -Y(reversed direction in that case, because Y was spatially flipped in the opposite direction)
        P: -Y -> X(reversed direction in that case too!)
        Y: X -> Z - direction not reversed

        => formula to get right rotations:
        -Rx -Rz Ry???

        rotationite vav Ogre3D's coordinate system sa RYP, ne RPY!!!, tova pak ne obiasniava rabotite???

        TODO: figure out kak sum mapnal samia URDF kam samia Ogre3D coordinate system i samite rotation frames!!!
    */

        Ogre::SceneNode* baseNode = _createBase(
            _scene_manager->getRootSceneNode(),
            Ogre::Vector3(0, 0, 0),
            Ogre::Quaternion(1, 0, 0, 0)
        );

        // rpy="0.0 0.0 0.0" xyz="0.0 0.0 0.1273"
        // URDF Z axis maps to Ogre3D Y axis?!
        // Y rotation axis in Ogre3D
        Ogre::SceneNode* shoulderNode = _createJoint(
            baseNode,
            Ogre::Vector3(0, 0.1273, 0),
            0.0f, 0.0f, 0.0f,
            Axis::Y,
            false,
            "shoulder",
            3,
            0
        );

        // rpy="0.0 1.570796325 0.0" xyz="0.0 0.220941 0.0" - ORIGINAL
        // rpy="0.0 0.0 -1.570796325" xyz="0.0 0.0 -0.220941" - TRANSFORMED
        // URDF Y axis maps to Ogre3D z axis and with negative angles and coordinates?!
        // negative Z rotation axis in Ogre3D???
        Ogre::SceneNode* upperarmNode = _createJoint(
            shoulderNode,
            Ogre::Vector3(0.0, 0.0, -0.220941),
            0.0f, 0.0f, -1.570796325,
            Axis::X,
            true,
            "upperarm",
            4,
            1
        );

        // rpy="0.0 0.0 0.0" xyz="0.0 -0.1719 0.612" -  ORIGINAL
        // rpy="0.0 0.0 0.0" xyz="0.0 0.612 0.1719" - TRANSFORMED FOR OGRE3D
        // negative Z rotation axis in Ogre3D???
        Ogre::SceneNode* forearmNode = _createJoint(
            upperarmNode,
            Ogre::Vector3(0.0, 0.612, 0.1719),
            0.0f, 0.0f, 0.0f,
            Axis::X,
            true,
            "forearm",
            5,
            2
        );

        // rpy="0.0 1.570796325 0.0" xyz="0.0 0.0 0.5723" - ORIGINAL
        // rpy="0.0 0.0 -1.570796325" xyz="0.0 0.5723 0.0" - TRANSFORMED
        // negative Z rotation axis in Ogre3D???
        Ogre::SceneNode* wrist1Node = _createJoint(
            forearmNode,
            Ogre::Vector3(0.0, 0.5723, 0.0),
            0.0, 0.0, -1.570796325,
            Axis::X,
            true,
            "wrist1_",
            3,
            3
        );

        // rpy="0.0 0.0 0.0" xyz="0.0 0.1149 0.0" - ORIGINAL
        // rpy="0.0 0.0 0.0" xyz="0.0 0.0 -0.1149" - TRANSFORMED
        // Y rotation axis
        Ogre::SceneNode* wrist2Node = _createJoint(
            wrist1Node,
            Ogre::Vector3(0.0, 0.0, -0.1149),
            0.0, 0.0, 0.0,
            Axis::Y,
            false,
            "wrist2_",
            3,
            4
        );

        // rpy="0.0 0.0 0.0" xyz="0.0 0.0 0.1157" - ORIGINAL
        // rpy="0.0 0.0 0.0" xyz="0.0 0.1157 0.0" - TRANSFORMED
        // negative Z rotation axis in Ogre3D???
        Ogre::SceneNode* wrist3Node = _createJoint(
            wrist2Node,
            Ogre::Vector3(0.0, 0.1157, 0.0),
            0.0, 0.0, 0.0,
            Axis::X,
            true,
            "wrist3",
            1,
            5
        );

        // Apply initial angles
        set_angles(_joint_angles);

		// Init collision
			// Load .obj collision meshes.
			// Get transform matrix from the Ogre3D objects(IT MUST BE THE GLOBAL TRANSFORM USING _getFullTransform!!!)
			// Apply the same transform matrix to the collision meshes(GLOBAL TRANSFORM USING _getFullTransform)
			// Add the collision meshes to _collision_manager.
	}

    vector<real_t> get_angles() const {
        return _joint_angles;
    }

    // TODO: WRITE MY OWN VERSION: https://forums.ogre3d.org/viewtopic.php?t=53647
    static void _DestroyAllAttachedMovableObjects( Ogre::SceneNode* i_pSceneNode )
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
    void update_collider_transforms() {
        for(size_t i = 0; i < 6; i++) {
            // Ogre::Vector3 translation;
            // Ogre::Vector3 scaling;
            // Ogre::Quaternion rotation;

            // _joint_nodes[i]->_getFullTransform().decomposition(translation, scaling, rotation); // It is exactly the same for the colliders

            // // Change to real_t!!!
            // // fcl::Transform3d transform =
            // //     fcl::Quaterniond(rotation.w, rotation.x, rotation.y, rotation.z) *
            // //     fcl::Translation3d(translation.x, translation.y, translation.z);
            
            // fcl::Transform3d transform =
            //     fcl::Translation3d(translation.x, translation.y, translation.z) *
            //     fcl::Quaterniond(rotation.w, rotation.x, rotation.y, rotation.z);
                

            // _collision_objects[i]->setTransform(transform);

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
        // _collision_manager->update();

        // Perform collision detection(TODO: OPTIMIZE)
        fcl::test::CollisionData<double> collision_data; // MANIPULATE .request FOR MORE CONTACT POINTS!!!
        // fcl::test::DistanceData<double> distance_data;

        collision_data.request.enable_contact = true; // kogato e false dava nepravilni results?? taka li shte e i sas collision samo mezdu 2 objects???
        // collision_data.request.num_max_contacts = 20000;
        // collision_data.request.num_max_contacts = 5000;
        const intmax_t NUMBER_OF_CONTACT_POINTS = 50;
        collision_data.request.num_max_contacts = NUMBER_OF_CONTACT_POINTS;
        // collision_data.result.
        // collision_data.request.
        
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

	void set_angle(size_t joint_index, real_t angle)
	{
		assert(joint_index < _joint_angles.size());
		// assert(/* angle in the range of [-PI; PI) */);

		_joint_angles[joint_index] = angle;
        
        // Apply the angle to the correct rotation frame, correct axis and correct direction
        real_t computed_angle = angle;
        if(_reverse_rotation_direction[joint_index]) {
            computed_angle = (-computed_angle);
        }

        _joint_nodes[joint_index]->resetOrientation();

        if(_axis_of_rotation[joint_index] == Axis::X) {
            _joint_nodes[joint_index]->roll(Ogre::Radian(computed_angle)); // Z axis : https://ogrecave.github.io/ogre/api/1.11/tiki-download_file_rotation_axis.png
        } else if(_axis_of_rotation[joint_index] == Axis::Y) {
            _joint_nodes[joint_index]->yaw(Ogre::Radian(computed_angle)); // Y axis : https://ogrecave.github.io/ogre/api/1.11/tiki-download_file_rotation_axis.png
        } else {
            _joint_nodes[joint_index]->pitch(Ogre::Radian(computed_angle)); // X axis : https://ogrecave.github.io/ogre/api/1.11/tiki-download_file_rotation_axis.png
        }

        _fk->set_angle(joint_index, angle);

        // IS THE INTERNAL AFFINE MATRIX UPDATED AFTER THIS??? It must be for the collision detection to work!!!

        // CALLING update_collider_transforms(); HERE WOULD BE TOO EXPENSIVE!!!

        // TODO
		// Update the collision manager transforms ( setTransform() http://docs.ros.org/fuerte/api/fcl/html/classfcl_1_1CollisionObject.html )
		// Force FCL to update the collision manager data structures! (use update(CollisionObject *updated_obj) for the particular collision manager http://docs.ros.org/fuerte/api/fcl/html/classfcl_1_1BroadPhaseCollisionManager.html) 
	}
	
	void rotate_joint(size_t joint_index, real_t angle)
	{
		// You want to call a private method instead, but that works for now
		set_angle(
			joint_index,
			_joint_angles[joint_index] + angle
		);

        update_collider_transforms();
	}
	
};

class ObjectSelectionController : public OgreBites::InputListener {
private:
    Ogre::SceneManager*   _scene_manager    = nullptr;
    Ogre::Camera*         _camera           = nullptr;
    OgreBites::CameraMan* _orbit_controller = nullptr;
    Ogre::RenderWindow*   _render_window    = nullptr;
    InputManager*         _input_manager    = nullptr;

    Ogre::SceneNode* _last_selection = nullptr;

    bool is_selection_mode_enabled = false;

    void _select_object(Ogre::SceneNode* node) {
        if(_last_selection) {
            _last_selection->showBoundingBox(false);
        }

        node->showBoundingBox(true);
        _last_selection = node;
    }

    void _select_object_through_viewport_raycast(
        decltype(OgreBites::MouseButtonEvent::x) x,
        decltype(OgreBites::MouseButtonEvent::y) y
    ) {
        // Raycast from viewport
        Ogre::RaySceneQuery *mRaySceneQuery = _scene_manager->createRayQuery(Ogre::Ray());
        mRaySceneQuery->setSortByDistance( true );

        Ogre::Ray scopeRay = _camera->getCameraToViewportRay(
            static_cast<double>(x)/_render_window->getWidth(),
            static_cast<double>(y)/_render_window->getHeight()
        );

        mRaySceneQuery->setRay(scopeRay);
        Ogre::RaySceneQueryResult& result = mRaySceneQuery->execute();
        
        if(result.size()) {

            // All of the objects placed in the scene are movable for easier implementation. Optimizations with static geometry are not needed
            _orbit_controller->setTarget(
                dynamic_cast<Ogre::SceneNode*>(result.at(0).movable->getParentNode())
            );

            // Show bounding box for indicating the selection
            _select_object(
                dynamic_cast<Ogre::SceneNode*>(result.at(0).movable->getParentNode())
            );

            // Force camera position update
            OgreBites::MouseMotionEvent simulated_motion_event;
            simulated_motion_event.type = OgreBites::MOUSEMOTION;
            simulated_motion_event.x = x;
            simulated_motion_event.y = y;
            simulated_motion_event.xrel = 0;
            simulated_motion_event.yrel = 0;
            _orbit_controller->mouseMoved(simulated_motion_event);
            cout << "I AM EXECUTIN THIS!!!" << endl;

            cout << ("ACTION: Switched camera target to " + result.at(0).movable->getParentNode()->getName()) << endl;
        }
    }
public:
    ObjectSelectionController(
        Ogre::SceneManager* scene_manager,
        Ogre::Camera* camera,
        OgreBites::CameraMan* orbit_controller,
        Ogre::RenderWindow* render_window,
        InputManager* input_manager
    ) :
        _scene_manager(scene_manager),
        _camera(camera),
        _orbit_controller(orbit_controller),
        _render_window(render_window),
        _input_manager(input_manager)
    {
        _input_manager->add_listener(this);
    }

    // Toggle the different axes for position/orientation
	bool keyPressed(const OgreBites::KeyboardEvent& evt) override {
        if (evt.keysym.sym == 'x' && evt.keysym.mod == 0 && (bool)evt.repeat == false) {
            is_selection_mode_enabled = true;
            cout << "ACTION: Selection mode enabled" << endl;
        }
    }
    
    // Toggle the different axes for position/orientation
	bool keyReleased(const OgreBites::KeyboardEvent& evt) override {
        if (evt.keysym.sym == 'x' && evt.keysym.mod == 0 && (bool)evt.repeat == false) {
            is_selection_mode_enabled = false;
            cout << "ACTION: Selection mode disabled" << endl;
        }
    }

    bool mousePressed(const OgreBites::MouseButtonEvent& evt) override {
        if(is_selection_mode_enabled) {
            _select_object_through_viewport_raycast(evt.x, evt.y);
        }

        return true;
    }

    // bool mouseMoved(const OgreBites::MouseMotionEvent &evt) override { 

    //     cout << "MOUSE MOVED DBEUG: " << endl;
    //     cout << evt.type << endl;
    //     cout << evt.windowID << endl;
    //     cout << evt.x << endl;
    //     cout << evt.xrel << endl;
    //     cout << evt.y << endl;
    //     cout << evt.yrel << endl;
    //     cout << endl;

    //     return true;
    // }
};

class JointController : public OgreBites::InputListener {
private:
    RobotModel* _model = nullptr;
    InputManager* _input_manager = nullptr;

    real_t _rotation_delta = 0.0;
    uintmax_t _selected_rotation_frame = 0;
public:
    JointController(RobotModel* model, InputManager* input_manager)
    :
        _model(model),
        _input_manager(input_manager)
    {
        _input_manager->add_listener(this);
    }

	bool keyPressed(const OgreBites::KeyboardEvent& evt) override {
        if (evt.keysym.sym >= '1' && evt.keysym.sym <= '6' && (bool)evt.repeat == false) {
            _selected_rotation_frame = evt.keysym.sym - '1';
        }
        if(evt.keysym.sym == '[') {
            _rotation_delta = -0.5;
        }

        if(evt.keysym.sym == ']') {
            _rotation_delta = 0.5;
        }
    }
    
	bool keyReleased(const OgreBites::KeyboardEvent& evt) override {
        if(evt.keysym.sym == '[' || evt.keysym.sym == ']') {
            _rotation_delta = 0.0;
        }
    }

	void frameRendered(const Ogre::FrameEvent& evt) override {
        // TODO: HANDLE JOINT LIMITS TOO!!!
        _model->rotate_joint(_selected_rotation_frame, _rotation_delta * evt.timeSinceLastFrame);
    }
};

class CartesianController : public OgreBites::InputListener {
private:
    // TODO: Rename delta to velocity and define exact units as m/s
    array<real_t, 3> _delta_position {{0, 0, 0}};
	array<real_t, 3> _delta_orientation {{0, 0, 0}};
	array<real_t, 3> _target_position {{0, 0, 0}};
	array<real_t, 3> _target_orientation {{0, 0, 0}};

    InverseKinematics* _ik_solver         = nullptr;
    RobotModel*        _robot_model       = nullptr;
    EndEffectorView*   _end_effector_view = nullptr;
    InputManager*      _input_manager     = nullptr;

    void _handle_key_press_and_release(const OgreBites::KeyboardEvent& evt, bool is_pressed) {
        const decltype(evt.keysym.mod) MOD_CTRL = 1; // TODO: DA NIAMAM PREDVID SHIFT??? vaobshte togava nuzno li e da definevam tova???

        real_t delta = 0.0;
        array<real_t, 3>* array_to_be_modified = nullptr;

        // Determine if the position or the orientation delta
        // needs to be modified
        if(evt.keysym.mod == 0) {
            array_to_be_modified = &_delta_position;
        } else if(evt.keysym.mod == MOD_CTRL) {
            array_to_be_modified = &_delta_orientation;
        }

        // Apply delta if pressed, else remove delta
        if(is_pressed) {
            if(evt.keysym.mod == 0) {
                delta = 0.1;
            } else if(evt.keysym.mod == MOD_CTRL) {
                delta = 0.4;
            }
        } else {
            delta = 0.0;
        }

        // NOTE: If Shift is released before W, A, S or D then the end effector will
        // continue to rotate
        // NOTE: If Shift is pressed after holding any of the W, A, S or D key and then the
        // W, A, S and D keys start to get released then the end effector will conitue
        // to change its position
        // TODO: A more robust system needs to be made that will generate a /pressed/ and /released/
        // event for keyboard combinations and even take into account order of pressing using a stack

        // Apply the delta to the corresponding axis
        if(array_to_be_modified != nullptr) {
            switch(evt.keysym.sym) {
                case 's': {
                    (*array_to_be_modified)[0] = delta;
                    break;
                }
                case 'w': {
                    (*array_to_be_modified)[0] = -delta;
                    break;
                }
                case 'a': {
                    (*array_to_be_modified)[1] = -delta;
                    break;
                }
                case 'd': {
                    (*array_to_be_modified)[1] = delta;
                    break;
                }
                case 'q': {
                    (*array_to_be_modified)[2] = -delta;
                    break;
                }
                case 'e': {
                    (*array_to_be_modified)[2] = delta;
                    break;
                }
                default: {
                    break;
                }
            }
        }
    }

    bool _is_delta_zero() {
        if(
            _delta_position[0] == 0 &&
            _delta_position[1] == 0 &&
            _delta_position[2] == 0 &&

            _delta_orientation[0] == 0 &&
            _delta_orientation[1] == 0 &&
            _delta_orientation[2] == 0
        ) {
            return true;
        } else {
            return false;
        }
    }

public:
    CartesianController(
        InverseKinematics* ik_solver,
        RobotModel* robot_model,
        EndEffectorView* end_effector_view,
        InputManager* input_manager
    ) :
        _ik_solver(ik_solver),
        _robot_model(robot_model),
        _end_effector_view(end_effector_view),
        _input_manager(input_manager)
    {
        _input_manager->add_listener(this);
    }
    
    // Toggle the different axes for position/orientation
	bool keyPressed(const OgreBites::KeyboardEvent& evt) override {
        _handle_key_press_and_release(evt, true);
    }
    
    // Toggle the different axes for position/orientation
	bool keyReleased(const OgreBites::KeyboardEvent& evt) override {
        _handle_key_press_and_release(evt, false);
    }

    // Increment the position/orientation axes using the deltas on every
    // frameRendered event. If the position or orientation is updated, because
    // the delta is different from zero then recalculate the IK. 
	void frameRendered (const Ogre::FrameEvent& evt) override {
        // TODO: Use this: evt.timeSinceLastFrame ???

        // Apply delta, perform IK, update RobotModel and EndEffectorView
        if(!_is_delta_zero()) {

            // Apply delta
            _target_position[0] += _delta_position[0] * evt.timeSinceLastFrame;
            _target_position[1] += _delta_position[1] * evt.timeSinceLastFrame;
            _target_position[2] += _delta_position[2] * evt.timeSinceLastFrame;
            
            _target_orientation[0] += _delta_orientation[0] * evt.timeSinceLastFrame;
            _target_orientation[1] += _delta_orientation[1] * evt.timeSinceLastFrame;
            _target_orientation[2] += _delta_orientation[2] * evt.timeSinceLastFrame;

            // Perform IK
            _ik_solver->set_target_position(_target_position);
            _ik_solver->set_target_orientation(_target_orientation);

            // Limit the scope of variables when setting the current angles
            {
                vector<real_t> current_angles = _robot_model->get_angles();
                array<real_t, 6> current_angles_static_array {0, 0, 0, 0, 0, 0};

                for(intmax_t i = 0; i < current_angles_static_array.size(); i++) {
                    current_angles_static_array[i] = current_angles[i];
                }

                _ik_solver->set_current_angles(current_angles_static_array);
            }

            array<real_t, 6> resulting_angles {0, 0, 0, 0, 0, 0};
            intmax_t error_code = _ik_solver->solve(resulting_angles);

            if(error_code == -1) {
                cout << _ik_solver->get_last_error() << endl;
            }

            if(error_code == 0) {
                // Apply IK joint angles
                _robot_model->set_angles(resulting_angles);
            }

            // Update the target end effector view
            // HACK: Transforming the coordinate system
            _end_effector_view->set_position(
                -(_target_position[0]),
                _target_position[2],
                _target_position[1]
            );

            // INCORRECT ORIENTATION!!!
            _end_effector_view->set_orientation(
                _target_orientation[0],
                _target_orientation[1],
                _target_orientation[2]
            );
        }
    }
};

// TODO: RENAME THIS CLASS!!!
class MyTestApp : public OgreBites::ApplicationContext, public OgreBites::InputListener
{
private:
    OgreBites::CameraMan* mCameraMan;
    Ogre::SceneManager* scnMgr;
    Ogre::Camera* cam;

    RobotModel* model;

    uintmax_t _rotation_frame_selection;
    // bool selectionMode = false;
    vector<real_t> ikPosition;
    vector<real_t> ikOrientation;
    EndEffectorView* _end_effector_view;
    ObjectSelectionController* _object_selector;
    InverseKinematics* _ik_solver;
    CartesianController* _cartesian_controller;
    JointController* _joint_controller;
    AxisHelperFactory* _axis_helper_factory;

    InputManager* _input_manager;
public:
    MyTestApp();
    void setup();

    bool keyPressed (const OgreBites::KeyboardEvent& evt) override;
    // void frameRendered (const Ogre::FrameEvent& evt) override;
    // bool keyReleased (const OgreBites::KeyboardEvent& evt) override;
    // bool mouseMoved (const OgreBites::MouseMotionEvent& evt) override;
    // bool mousePressed (const OgreBites::MouseButtonEvent& evt) override;
    // bool mouseReleased (const OgreBites::MouseButtonEvent& evt) override;
    // bool mouseWheelRolled (const OgreBites::MouseWheelEvent& evt) override;
    // bool touchMoved (const OgreBites::TouchFingerEvent& evt) override;
    // bool touchPressed (const OgreBites::TouchFingerEvent& evt) override;
    // bool touchReleased (const OgreBites::TouchFingerEvent& evt) override;
};

MyTestApp::MyTestApp() : OgreBites::ApplicationContext("OgreTutorialApp"), _rotation_frame_selection(0), ikPosition{1, 0, 0}, ikOrientation{0, 0, 0}
{
}

bool MyTestApp::keyPressed(const OgreBites::KeyboardEvent& evt)
{
    if (evt.keysym.sym == 'v') {
        model->toggle_collider_visibility();
    }

    if (evt.keysym.sym == 'b') {
        model->toggle_visual_mesh_visibility();

        // HACK: Because toggle_visual_mesh_visibility makes also the collision meshes invisible they need to
        // be retoggled with the following hack:
        model->toggle_collider_visibility();
        model->toggle_collider_visibility();
    }

    if (evt.keysym.sym == OgreBites::SDLK_ESCAPE)
    {
        getRoot()->queueEndRendering();
    }

    return true;
}

void MyTestApp::setup(void)
{
    // do not forget to call the base first
    OgreBites::ApplicationContext::setup();

    // get a pointer to the already created root
    Ogre::Root* root = getRoot();
    scnMgr = root->createSceneManager();

    // Create axis helper factory
    _axis_helper_factory = new AxisHelperFactory {scnMgr};

    // register our scene with the RTSS
    Ogre::RTShader::ShaderGenerator* shadergen = Ogre::RTShader::ShaderGenerator::getSingletonPtr();
    shadergen->addSceneManager(scnMgr);

    // Create end effector visualization
    _end_effector_view = new EndEffectorView {scnMgr, _axis_helper_factory};

    // Contact points material for collision detection debugging
    Ogre::MaterialPtr contactPointsMaterial = Ogre::MaterialManager::getSingleton().create("contact_points_material"s, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
    contactPointsMaterial->setDiffuse(Ogre::ColourValue(0, 0, 1));
    // Make the ambient light have less influence on this object and remove the red and green
    // influence from the ambient color due to the diffuse not being red or green, hence the object
    // generally not being able to scatter/reflect this color.
    // TODO: Maybe derive this as a scaling of the diffuse color???
    contactPointsMaterial->setAmbient(Ogre::ColourValue(0, 0, 1)); 

    // without light we would just get a black screen
    Ogre::Light* light = scnMgr->createLight("MainLight");
    Ogre::SceneNode* lightNode = scnMgr->getRootSceneNode()->createChildSceneNode();
    lightNode->setPosition(0, 40, 15);
    lightNode->attachObject(light);

    // Opacity component seems to be ignored? When is it used?
    scnMgr->setAmbientLight(Ogre::ColourValue(0.2, 0.2, 0.2, 0));

    Ogre::Light* lightSecondary = scnMgr->createLight("SecondaryLight"); // Do I need to create another light??? Maybe attach it to another scene node???
    Ogre::SceneNode* lightNodeSecondary = scnMgr->getRootSceneNode()->createChildSceneNode();
    lightNodeSecondary->setPosition(0, 40, -15);
    lightNodeSecondary->attachObject(lightSecondary);

    // also need to tell where we are
    Ogre::SceneNode* camNode = scnMgr->getRootSceneNode()->createChildSceneNode();
    camNode->setPosition(0, 0, 1);
    camNode->lookAt(Ogre::Vector3(0, 0, -1), Ogre::Node::TS_PARENT);

    // create the camera
    cam = scnMgr->createCamera("myCam");
    cam->setNearClipDistance(5); // specific to this sample
    cam->setAutoAspectRatio(true);
    cam->setNearClipDistance(0.01);
    camNode->attachObject(cam);

    // and tell it to render into the main window
    getRenderWindow()->addViewport(cam);

    // Create and display grid
    Ogre::MeshManager* meshMgr = this->getRoot()->getMeshManager();
    Ogre::Plane* plane = new Ogre::Plane(Ogre::Vector3(0, 1, 0), 0);
    Ogre::MeshPtr gridMesh = meshMgr->createPlane("grid", "Essential", *plane, 10, 10, 100, 100, true, 1, 1.0f, 1.0f, Ogre::Vector3(0, 0, 1)); // Essential corresponds to the groups found in resources.cfg!!!

    Ogre::MaterialPtr wireframeMat = Ogre::MaterialManager::getSingleton().create("white_grid", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
	wireframeMat->setDiffuse(Ogre::ColourValue(1,1,1));
	wireframeMat->getTechnique(0)->getPass(0)->setPolygonMode(Ogre::PM_WIREFRAME);
    wireframeMat->setCullingMode(Ogre::CullingMode::CULL_NONE); // Two-sided rendering
    
    Ogre::Entity* gridEntity = scnMgr->createEntity("grid");
    gridEntity->setMaterial(wireframeMat);
    Ogre::SceneNode* gridNode = scnMgr->getRootSceneNode()->createChildSceneNode("grid"s);
    gridNode->attachObject(gridEntity);

    // Camera orbit controller
    mCameraMan = new OgreBites::CameraMan(camNode); // Pointer will be lost!
    mCameraMan->setStyle(OgreBites::CameraStyle::CS_ORBIT);
    // mCameraMan->setTarget(node1);
    // selectObject(node1);
    mCameraMan->setYawPitchDist(Ogre::Radian(-0.5), Ogre::Radian(0.25), 0.5); // Default camera position
    
    //addInputListener(mCameraMan); // Wait, does this make my InputManager obsolete??? Or maybe not, because it's more
    // specialized and I won't inject objects with more responsibilities in my components???
    
    // Create input manager
    _input_manager = new InputManager();
    _input_manager->add_listener(this);

    // register for input events
    addInputListener(_input_manager);

    //fcl::BroadPhaseCollisionManager<double>* collision_manager = new fcl::DynamicAABBTreeCollisionManager<double>();
    fcl::BroadPhaseCollisionManager<double>* collision_manager = new fcl::NaiveCollisionManager<double>();
    CollisionSelectionManager* collision_selection_manager = new CollisionSelectionManager();

    fcl::aligned_vector<fcl::Transform3d>* local_reference_frame = new fcl::aligned_vector<fcl::Transform3d> {
        fcl::Translation3d(0, 0.1273, 0) *
        fcl::AngleAxisd(0.0, fcl::Vector3d::UnitX()) *
        fcl::AngleAxisd(0.0, fcl::Vector3d::UnitY()) *
        fcl::AngleAxisd(0.0, fcl::Vector3d::UnitZ()),
        
        fcl::Translation3d(0.0, 0.0, -0.220941) *
        fcl::AngleAxisd(0.0, fcl::Vector3d::UnitX()) *
        fcl::AngleAxisd(0.0, fcl::Vector3d::UnitY()) *
        fcl::AngleAxisd(-1.570796325, fcl::Vector3d::UnitZ()), // Replace with -M_PI_2

        fcl::Translation3d(0.0, 0.612, 0.1719) *
        fcl::AngleAxisd(0.0, fcl::Vector3d::UnitX()) *
        fcl::AngleAxisd(0.0, fcl::Vector3d::UnitY()) *
        fcl::AngleAxisd(0.0, fcl::Vector3d::UnitZ()),

        fcl::Translation3d(0.0, 0.5723, 0.0) *
        fcl::AngleAxisd(0.0, fcl::Vector3d::UnitX()) *
        fcl::AngleAxisd(0.0, fcl::Vector3d::UnitY()) *
        fcl::AngleAxisd(-1.570796325, fcl::Vector3d::UnitZ()),

        fcl::Translation3d(0.0, 0.0, -0.1149) *
        fcl::AngleAxisd(0.0, fcl::Vector3d::UnitX()) *
        fcl::AngleAxisd(0.0, fcl::Vector3d::UnitY()) *
        fcl::AngleAxisd(0.0, fcl::Vector3d::UnitZ()),

        fcl::Translation3d(0.0, 0.1157, 0.0) *
        fcl::AngleAxisd(0.0, fcl::Vector3d::UnitX()) *
        fcl::AngleAxisd(0.0, fcl::Vector3d::UnitY()) *
        fcl::AngleAxisd(0.0, fcl::Vector3d::UnitZ()),
    };

    vector<Axis> rotation_axis {
        Axis::Y,
        Axis::X,
        Axis::X,
        Axis::X,
        Axis::Y,
        Axis::X
    };

    vector<bool> reverse_rotation_direction {
        false,
        true,
        true,
        true,
        false,
        true
    };

    ForwardKinematics* fk = new ForwardKinematics {
        local_reference_frame,
        rotation_axis,
        reverse_rotation_direction
    };

    model = new RobotModel{scnMgr, collision_manager, collision_selection_manager, fk};

    // MUST BE BEFORE OBJECT SELECTOR!!! Otherwise the MousePressed event of mCameraMan will be processed
    // after the manual mouseMoved event, which means that it won't be taken into account, because the
    // mousePressed event needs to be before it!!!
    // NOTE: Could be avoided by simulating the MousePressed and MouseReleased events!!!
    _input_manager->add_listener(mCameraMan);
    _object_selector = new ObjectSelectionController {
        scnMgr,
        cam,
        mCameraMan,
        getRenderWindow(),
        _input_manager
    };

    _ik_solver = new InverseKinematics {};
    _cartesian_controller = new CartesianController {
        _ik_solver, model,
        _end_effector_view,
        _input_manager
    };

    _joint_controller = new JointController {
        model,
        _input_manager
    };

    // // Create world axes and attach them to the scene
    // Ogre::ManualObject* axis;

    // if(!scnMgr->hasManualObject("axis"s)) {
    //     axis = scnMgr->createManualObject("axis"s); 
    //     Ogre::MaterialPtr x_axis_material = Ogre::MaterialManager::getSingleton().create("x_axis_material"s, "General"s); 
    //     x_axis_material->setReceiveShadows(false); 
    //     x_axis_material->getTechnique(0)->setLightingEnabled(true); 
    //     x_axis_material->getTechnique(0)->getPass(0)->setDiffuse(0,0,1,0); 
    //     x_axis_material->getTechnique(0)->getPass(0)->setAmbient(0,0,1); 
    //     x_axis_material->getTechnique(0)->getPass(0)->setSelfIllumination(0,0,1); 

    //     axis->begin("x_axis_material"s, Ogre::RenderOperation::OT_LINE_LIST);
    //     axis->position(0, 0, 0);
    //     axis->position(0, 0, 1);
    //     axis->end(); 
    // } else {
    //     axis = scnMgr->getManualObject("axis"s); 
    // }
    
    // Ogre::SceneNode* axis_node = scnMgr->getRootSceneNode()->createChildSceneNode("x_axis_node"s); 
    // axis_node->setPosition(0, 0.00001, 0); // So that it won't overlap with the grid
    // axis_node->attachObject(axis);
    
    // Create world axes
    Ogre::SceneNode* axes_helper_node = _axis_helper_factory->get_axes_node("axes_helper"s);
    axes_helper_node->setPosition(0.0001, 0, 0); // Improvement on flickering issue from interleaving with the grid. POSSIBLE SOLUTION: use cylinders instead of lines
    axes_helper_node->setScale(5, 5, 5);
    scnMgr->getRootSceneNode()->addChild(axes_helper_node);
    
    // JUST FOR TESTING!!!
    // AxisHelperFactory factory { scnMgr };
    // Ogre::SceneNode* axes_helper_node = factory.get_axes_node("axes_helper"s); // USE axes_helper as a name!!!

    // scnMgr->getRootSceneNode()->addChild(axes_helper_node);

    // // Try retrieving the same object multiple times and reattaching it again!!!
    // axes_helper_node = factory.get_axes_node("axes_helper"s);
    // scnMgr->getRootSceneNode()->addChild(axes_helper_node);

    // Align Ogre3D and URDF Axes:

    // Using quaternions
    // TODO: USE THIS TO FIX THE FRAME INCONSISTENCY PROBLEMS!!!
    // Don't use DHParams with KDL, but use the URDF reference frames
    // instead. Also use the the reference frames in Ogre3D by applying the
    // correction beforehand!!!
    // axes_helper_node_urdf->rotate(
    //     Ogre::Quaternion(Ogre::Radian(-M_PI_2), {1, 0, 0}) *
    //     Ogre::Quaternion(Ogre::Radian(M_PI), {0, 0, 1})
    // );
    
    // Using Euler angles
    // axes_helper_node_urdf->pitch(Ogre::Radian(-M_PI_2));
    // axes_helper_node_urdf->roll(Ogre::Radian(M_PI));
}

int main(int argc, char *argv[])
{
    MyTestApp app;
    app.initApp();
    app.getRoot()->startRendering();
    app.closeApp();
    return 0;
}
