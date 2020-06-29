#include <iostream>
#include <string>

#include <Ogre.h>
#include <OgreApplicationContext.h>
#include <OgreCameraMan.h>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/random_generator.hpp>
#include <boost/uuid/uuid_io.hpp>

#include "util.h"
#include "axes_helper_factory.h"
#include "input_manager.h"
#include "end_effector_view.h"
#include "forward_kinematics.h"
#include "inverse_kinematics.h"
#include "collision_selection_manager.h"
#include "object_selection_controller.h"
#include "robot_model.h"
#include "joint_controller.h"
#include "cartesian_controller.h"
#include "robot_view.h"
#include "collision_view.h"
#include "collision_model.h"
#include "point_cloud_view.h"
#include "collision_controller.h"
#include "box_collider.h"
#include "graph_view.h"
#include "path_view.h"
#include "motion_planner.h"
#include "trajectory_controller.h"
#include "naive_nearest_neighbour.h"
#include "visibility_controller.h"
#include "motion_planning_controller.h"

using namespace std;

class RoboMotionPlanner : public OgreBites::ApplicationContext, public OgreBites::InputListener {
private:
    Ogre::SceneManager* _scene_manager { nullptr };
    Ogre::Camera* _camera { nullptr };

    RobotView*                                _robot_view              { nullptr };
    CollisionView*                            _collision_view          { nullptr };
    ForwardKinematics*                        _fk                      { nullptr };
    InverseKinematics*                        _ik_solver               { nullptr };
    CollisionModel*                           _collision_model         { nullptr };
    CollisionController*                      _collision_controller    { nullptr };
    PointCloudView*                           _contact_points_view     { nullptr };
    fcl::BroadPhaseCollisionManager<real_t>*  _scene_collision_manager { nullptr };
    vector<BoxCollider>                       _box_colliders;
    PointCloudView*                           _sampled_points_view     { nullptr };
    GraphView*                                _config_space_graph_view { nullptr };
    PathView*                                 _computed_path_view      { nullptr };

    AxisHelperFactory*         _axis_helper_factory  { nullptr };
    EndEffectorView*           _end_effector_view    { nullptr };
    
    INearestNeighbour*         _nearest_neighbour                { nullptr };
    MotionPlanner*             _motion_planner                   { nullptr };
    ObjectSelectionController* _object_selection_controller      { nullptr };
    CartesianController*       _cartesian_controller             { nullptr };
    JointController*           _joint_controller                 { nullptr };
    TrajectoryController*      _trajectory_controller            { nullptr };
    VisibilityController*      _visibility_controller            { nullptr };
    MotionPlanningController*  _motion_planning_controller       { nullptr };
    OgreBites::CameraMan*      _orbit_controller                 { nullptr };

    InputManager* _input_manager { nullptr };
public:
    RoboMotionPlanner();
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

RoboMotionPlanner::RoboMotionPlanner()
: 
    OgreBites::ApplicationContext { "OgreTutorialApp"s }
{}

bool RoboMotionPlanner::keyPressed(const OgreBites::KeyboardEvent& evt)
{
    if (evt.keysym.sym == OgreBites::SDLK_ESCAPE) {
        getRoot()->queueEndRendering();
    }

    return true;
}

void RoboMotionPlanner::setup(void)
{
    // do not forget to call the base first
    OgreBites::ApplicationContext::setup();

    // get a pointer to the already created root
    Ogre::Root* root = getRoot();
    _scene_manager = root->createSceneManager();

    // UUID generator
    boost::uuids::basic_random_generator<boost::mt19937> uuid_generator;

    // Create axis helper factory
    _axis_helper_factory = new AxisHelperFactory {_scene_manager};

    // register our scene with the RTSS
    Ogre::RTShader::ShaderGenerator* shadergen = Ogre::RTShader::ShaderGenerator::getSingletonPtr();
    shadergen->addSceneManager(_scene_manager);

    // Create end effector visualization
    _end_effector_view = new EndEffectorView {
        _scene_manager,
        _axis_helper_factory
    };

    // TODO: REMOVE THIS MATERIAL CODE, already available elsewhere!!!
    // Contact points material for collision detection debugging
    Ogre::MaterialPtr contactPointsMaterial = Ogre::MaterialManager::getSingleton().create("contact_points_material"s, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
    contactPointsMaterial->setDiffuse(Ogre::ColourValue(0, 0, 1));
    // Make the ambient light have less influence on this object and remove the red and green
    // influence from the ambient color due to the diffuse not being red or green, hence the object
    // generally not being able to scatter/reflect this color.
    // TODO: Maybe derive this as a scaling of the diffuse color???
    contactPointsMaterial->setAmbient(Ogre::ColourValue(0, 0, 1)); 

    // without light we would just get a black screen
    Ogre::Light* light = _scene_manager->createLight("MainLight");
    Ogre::SceneNode* lightNode = _scene_manager->getRootSceneNode()->createChildSceneNode();
    lightNode->setPosition(0, 40, 15);
    lightNode->attachObject(light);

    // Opacity component seems to be ignored? When is it used?
    _scene_manager->setAmbientLight(Ogre::ColourValue(0.2, 0.2, 0.2, 0));

    Ogre::Light* lightSecondary = _scene_manager->createLight("SecondaryLight"); // Do I need to create another light??? Maybe attach it to another scene node???
    Ogre::SceneNode* lightNodeSecondary = _scene_manager->getRootSceneNode()->createChildSceneNode();
    lightNodeSecondary->setPosition(0, 40, -15);
    lightNodeSecondary->attachObject(lightSecondary);

    // also need to tell where we are
    Ogre::SceneNode* camNode = _scene_manager->getRootSceneNode()->createChildSceneNode();
    camNode->setPosition(0, 0, 1);
    camNode->lookAt(Ogre::Vector3(0, 0, -1), Ogre::Node::TS_PARENT);

    // create the camera
    _camera = _scene_manager->createCamera("myCam");
    _camera->setNearClipDistance(5); // specific to this sample
    _camera->setAutoAspectRatio(true);
    _camera->setNearClipDistance(0.01);
    camNode->attachObject(_camera);

    // and tell it to render into the main window
    getRenderWindow()->addViewport(_camera);

    // Create and display grid
    Ogre::MeshManager* meshMgr = this->getRoot()->getMeshManager();
    Ogre::Plane* plane = new Ogre::Plane(Ogre::Vector3(0, 1, 0), 0);
    Ogre::MeshPtr gridMesh = meshMgr->createPlane("grid", "Essential", *plane, 10, 10, 100, 100, true, 1, 1.0f, 1.0f, Ogre::Vector3(0, 0, 1)); // Essential corresponds to the groups found in resources.cfg!!!

    Ogre::MaterialPtr wireframeMat = Ogre::MaterialManager::getSingleton().create("white_grid", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
	wireframeMat->setDiffuse(Ogre::ColourValue(1,1,1));
	wireframeMat->getTechnique(0)->getPass(0)->setPolygonMode(Ogre::PM_WIREFRAME);
    wireframeMat->setCullingMode(Ogre::CullingMode::CULL_NONE); // Two-sided rendering
    
    Ogre::Entity* gridEntity = _scene_manager->createEntity("grid");
    gridEntity->setMaterial(wireframeMat);
    Ogre::SceneNode* gridNode = _scene_manager->getRootSceneNode()->createChildSceneNode("grid"s);
    gridNode->attachObject(gridEntity);

    // Camera orbit controller
    _orbit_controller = new OgreBites::CameraMan(camNode); // Pointer will be lost!
    _orbit_controller->setStyle(OgreBites::CameraStyle::CS_ORBIT);
    _orbit_controller->setYawPitchDist(Ogre::Radian(-0.5), Ogre::Radian(0.25), 0.5); // Default camera position
    
    // Create input manager
    _input_manager = new InputManager{};
    _input_manager->add_listener(this);

    // register for input events
    addInputListener(_input_manager);

    fcl::BroadPhaseCollisionManager<double>* collision_manager = new fcl::NaiveCollisionManager<double>{};
    CollisionSelectionManager* collision_selection_manager = new CollisionSelectionManager{};

    fcl::aligned_vector<fcl::Transform3d>* local_reference_frame =
        new fcl::aligned_vector<fcl::Transform3d> {
            // rpy="0.0 0.0 0.0" xyz="0.0 0.0 0.1273" - ORIGINAL
            // rpy="0.0 0.0 0.0" xyz="0.0 0.1273 0.0" - TRANSFORMED
            fcl::Translation3d(0, 0.1273, 0) *
            fcl::AngleAxisd(0.0, fcl::Vector3d::UnitX()) *
            fcl::AngleAxisd(0.0, fcl::Vector3d::UnitY()) *
            fcl::AngleAxisd(0.0, fcl::Vector3d::UnitZ()),
            
            // rpy="0.0 1.570796325 0.0" xyz="0.0 0.220941 0.0" - ORIGINAL
            // rpy="0.0 0.0 -1.570796325" xyz="0.0 0.0 -0.220941" - TRANSFORMED
            fcl::Translation3d(0.0, 0.0, -0.220941) *
            fcl::AngleAxisd(0.0, fcl::Vector3d::UnitX()) *
            fcl::AngleAxisd(0.0, fcl::Vector3d::UnitY()) *
            fcl::AngleAxisd(-1.570796325, fcl::Vector3d::UnitZ()), // Replace with -M_PI_2

            // rpy="0.0 0.0 0.0" xyz="0.0 -0.1719 0.612" -  ORIGINAL
            // rpy="0.0 0.0 0.0" xyz="0.0 0.612 0.1719" - TRANSFORMED FOR OGRE3D
            fcl::Translation3d(0.0, 0.612, 0.1719) *
            fcl::AngleAxisd(0.0, fcl::Vector3d::UnitX()) *
            fcl::AngleAxisd(0.0, fcl::Vector3d::UnitY()) *
            fcl::AngleAxisd(0.0, fcl::Vector3d::UnitZ()),

            // rpy="0.0 1.570796325 0.0" xyz="0.0 0.0 0.5723" - ORIGINAL
            // rpy="0.0 0.0 -1.570796325" xyz="0.0 0.5723 0.0" - TRANSFORMED
            fcl::Translation3d(0.0, 0.5723, 0.0) *
            fcl::AngleAxisd(0.0, fcl::Vector3d::UnitX()) *
            fcl::AngleAxisd(0.0, fcl::Vector3d::UnitY()) *
            fcl::AngleAxisd(-1.570796325, fcl::Vector3d::UnitZ()),

            // rpy="0.0 0.0 0.0" xyz="0.0 0.1149 0.0" - ORIGINAL
            // rpy="0.0 0.0 0.0" xyz="0.0 0.0 -0.1149" - TRANSFORMED
            fcl::Translation3d(0.0, 0.0, -0.1149) *
            fcl::AngleAxisd(0.0, fcl::Vector3d::UnitX()) *
            fcl::AngleAxisd(0.0, fcl::Vector3d::UnitY()) *
            fcl::AngleAxisd(0.0, fcl::Vector3d::UnitZ()),

            // rpy="0.0 0.0 0.0" xyz="0.0 0.0 0.1157" - ORIGINAL
            // rpy="0.0 0.0 0.0" xyz="0.0 0.1157 0.0" - TRANSFORMED
            fcl::Translation3d(0.0, 0.1157, 0.0) *
            fcl::AngleAxisd(0.0, fcl::Vector3d::UnitX()) *
            fcl::AngleAxisd(0.0, fcl::Vector3d::UnitY()) *
            fcl::AngleAxisd(0.0, fcl::Vector3d::UnitZ()),
        }
    ;

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

    _fk = new ForwardKinematics {
        local_reference_frame,
        rotation_axis,
        reverse_rotation_direction
    };

    // _robot_model = new RobotModel {
    //     _scene_manager,
    //     collision_manager,
    //     collision_selection_manager,
    //     fk
    // };

    _robot_view = new RobotView {
        _scene_manager
    };

    _collision_view = new CollisionView {
        _scene_manager
    };
    
    _collision_model = new CollisionModel {};

    boost::uuids::uuid contact_points_uuid = uuid_generator();
    _contact_points_view = new PointCloudView {
        boost::uuids::to_string(contact_points_uuid),
        _scene_manager
    };

    boost::uuids::uuid sampled_points_uuid = uuid_generator();
    _sampled_points_view = new PointCloudView {
        boost::uuids::to_string(sampled_points_uuid),
        _scene_manager
    };
    
    _scene_collision_manager = new fcl::NaiveCollisionManager<real_t>{};

    _collision_controller = new CollisionController {
        _collision_model,
        _contact_points_view,
        _scene_collision_manager
    };
    _input_manager->add_listener(_collision_controller);

    // _box_colliders.push_back(
    //     BoxCollider(
    //         _scene_manager,
    //         _scene_collision_manager,
    //         {{ 0.1, 0.1, 0.1 }},
    //         {{ 0, 0.07071067811, 0.3 }},
    //         {{ 0.924, 0, 0, 0.383 }} // TODO: Set the quaternion to the right values, test it out!!!
    //     )
    // );

    boost::uuids::uuid floor_collider_uuid = uuid_generator();
     _box_colliders.push_back(
        BoxCollider(
            boost::uuids::to_string(floor_collider_uuid),
            _scene_manager,
            _scene_collision_manager,
            {{ 2, 0.01, 2-0.5 }},
            {{ 0, -0.007, 1 }},
            {{ 1, 0, 0, 0 }} // TODO: Set the quaternion to the right values, test it out!!!
        )
    );

    boost::uuids::uuid separator_collider_uuid = uuid_generator();
    _box_colliders.push_back(
        BoxCollider(
            boost::uuids::to_string(separator_collider_uuid),
            _scene_manager,
            _scene_collision_manager,
            {{ 0.01, 0.5, 2 }},
            {{ 0, 0.25, 1.35 }},
            {{ 1, 0, 0, 0 }} // TODO: Set the quaternion to the right values, test it out!!!
        )
    );

    boost::uuids::uuid config_space_graph_uuid = uuid_generator();
    _config_space_graph_view = new GraphView {
        boost::uuids::to_string(config_space_graph_uuid),
        _scene_manager,
        Ogre::ColourValue(1, 1, 0, 0)
    };

    // TEST
    // vector<GraphEdge> edges {
    //     { {{ 1, 1, 1 }}, {{ -1, -1, -1 }} },
    //     { {{ 1, 1, -1 }}, {{ -1, -1, 1 }} },
    //     { {{ -1, -1, 1 }}, {{ 0, -1, 1 }} },
    // };
    // _config_space_graph_view->set_edges(edges);
    // _config_space_graph_view->set_edges(edges);

    boost::uuids::uuid path_view_uuid = uuid_generator();
    _computed_path_view = new PathView {
        boost::uuids::to_string(path_view_uuid),
        _scene_manager,
        Ogre::ColourValue(1, 0, 0, 0)
    };

    vector<array<real_t, 3>> points {
        {{  1,  1, -1 }},
        {{ -1, -1,  1 }},
        {{  0, -1,  1 }}
    };
    _computed_path_view->set_points(points);
    // _computed_path_view->set_points(points);

    // boost::uuids::uuid computed_path_uuid = uuid_generator();
    // _computed_path_view = new GraphView {
    //     boost::uuids::to_string(computed_path_uuid),
    //     _scene_manager,
    //     Ogre::ColourValue(1, 0, 0, 0)
    // };

    // vector<GraphEdge> edges_computed_path_test {
    //     { {{ 1, 1, 1 }}, {{ -1, -1, -1 }} }
    // };
    // _computed_path_view->set_edges(edges_computed_path_test);

    // TEST OGRE PROCEDURAL
    // Procedural::Path p = Procedural::Path()
    //     .addPoint(1, 1, -1)
    //     .addPoint(-1, -1, 1)
    //     .addPoint(0, -1, 1);

    // Procedural::Shape s = Procedural::CircleShape().setRadius(0.005).realizeShape();
    // Procedural::Extruder().setExtrusionPath(&p).setShapeToExtrude(&s).realizeMesh("extrudedMesh");

    // // Procedural::SphereGenerator().setRadius(5.f).setUTile(.5f).realizeMesh("mySphere"s);
    // // Ogre::Entity* sphere_entity = _scene_manager->createEntity("mySphere"s);
    // Ogre::Entity* sphere_entity = _scene_manager->createEntity("extrudedMesh"s);
    // Ogre::SceneNode* sphere_scene_node =
    //     _scene_manager->getRootSceneNode()->createChildSceneNode("sphere_procedural"s);
    // sphere_scene_node->attachObject(sphere_entity);

    // Initialize robot_view and collision_view, TODO: initialize collision_model
    for(intmax_t i = 0; i < 6; i++) {
        _robot_view->set_transform(i, _fk->get_transform(i));
        _collision_view->set_transform(i, _fk->get_transform(i));
    }

    // MUST BE BEFORE OBJECT SELECTOR!!! Otherwise the MousePressed event of mCameraMan will be processed
    // after the manual mouseMoved event, which means that it won't be taken into account, because the
    // mousePressed event needs to be before it!!!
    // NOTE: Could be avoided by simulating the MousePressed and MouseReleased events!!!
    _input_manager->add_listener(_orbit_controller);
    _object_selection_controller = new ObjectSelectionController {
        _scene_manager,
        _camera,
        _orbit_controller,
        getRenderWindow(),
        _input_manager
    };

    _ik_solver = new InverseKinematics {};
    _cartesian_controller = new CartesianController {
        _fk,
        _ik_solver,
        _robot_view,
        _collision_view,
        _collision_model,
        _collision_controller,
        _end_effector_view,
        _input_manager
    };

    _joint_controller = new JointController {
        _robot_view,
        _collision_view,
        _fk,
        _input_manager,
        _end_effector_view
    };

    _trajectory_controller = new TrajectoryController {
        _fk,
        _robot_view,
        _collision_view,
        _collision_model,
        _collision_controller,
        _end_effector_view,
        _input_manager,
        0.2
    };
    
    // Create world axes
    Ogre::SceneNode* axes_helper_node = _axis_helper_factory->get_axes_node("axes_helper"s);
    axes_helper_node->setPosition(0.0001, 0, 0); // Improvement on flickering issue from interleaving with the grid. POSSIBLE SOLUTION: use cylinders instead of lines
    axes_helper_node->setScale(5, 5, 5);    
    _scene_manager->getRootSceneNode()->addChild(axes_helper_node);
    
    // JUST FOR TESTING!!!
    // AxisHelperFactory factory { _scene_manager };
    // Ogre::SceneNode* axes_helper_node = factory.get_axes_node("axes_helper"s); // USE axes_helper as a name!!!

    // _scene_manager->getRootSceneNode()->addChild(axes_helper_node);

    // // Try retrieving the same object multiple times and reattaching it again!!!
    // axes_helper_node = factory.get_axes_node("axes_helper"s);
    // _scene_manager->getRootSceneNode()->addChild(axes_helper_node);

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

    // Initialize motion planner
    _nearest_neighbour = new NaiveNearestNeighbour {};

    /** THE FOLLOWING MotionPlanner SETUP IS FOR DEMONSTRATION PURPOSES LATER ON!!! **/
    _motion_planner = new MotionPlanner {
        _collision_model,
        _scene_collision_manager,
        _fk,
        _ik_solver,
        _nearest_neighbour,
        {{
            // TODO: VISUALIZE!!!
            {{ 0.6, 0, 0 }},
            {{ -0.6, 0.8, 0.6 }},
        }},
        30, // ik_sample_count
        50, // sample_count
        0.01, // step_size
        0.5, // nearest_neighbour_radius TODO: HOW MUCH?! OLD: 0.05
        0.01, // min_distance_from_obstacle
        0.05 // max_error
    };

    _motion_planning_controller = new MotionPlanningController {
        _cartesian_controller,
        _motion_planner,
        _sampled_points_view,
        _config_space_graph_view,
        _computed_path_view,
        _trajectory_controller,
        _input_manager
    };

    /** GREAT SETUP FOR TESTING PATH OPTIMIZER **/
    // _motion_planner = new MotionPlanner {
    //     _collision_model,
    //     _scene_collision_manager,
    //     _fk,
    //     _ik_solver,
    //     _nearest_neighbour,
    //     {{
    //         // TODO: VISUALIZE!!!
    //         {{ 0.6, 0, 0 }},
    //         {{ -0.6, 0.8, 0.6 }},
    //     }},
    //     30, // ik_sample_count
    //     2000, // sample_count
    //     0.01, // step_size
    //     0.1, // nearest_neighbour_radius TODO: HOW MUCH?! OLD: 0.05
    //     0.01, // min_distance_from_obstacle
    //     0.05 // max_error
    // };

    _visibility_controller = new VisibilityController {
        _collision_view,
        _config_space_graph_view,
        _sampled_points_view,
        _input_manager
    };
}

int main(int argc, char *argv[])
{
    RoboMotionPlanner* robo_motion_planner = new RoboMotionPlanner {};

    robo_motion_planner->initApp();
    robo_motion_planner->getRoot()->startRendering();
    robo_motion_planner->closeApp();

    return 0;
}
