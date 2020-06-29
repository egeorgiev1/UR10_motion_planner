#include "test_fcl_utility.h"

#include "collision_controller.h"

#include <iostream>
using namespace std;

CollisionController::CollisionController(
    CollisionModel* robot_collision_model,
    PointCloudView* point_cloud_view,
    fcl::BroadPhaseCollisionManager<double>* scene_collision_manager
) :
    _robot_collision_model(robot_collision_model),
    _point_cloud_view(point_cloud_view),
    _scene_collision_manager(scene_collision_manager)
{}

void CollisionController::schedule_collision_detection() {
    _must_perform_collision_detection = true;
}

void CollisionController::frameRendered(const Ogre::FrameEvent& evt) {
    if(_must_perform_collision_detection) {
        _must_perform_collision_detection = false;

        // Perform collision detection here, display contact points (TODO: BE ABLE TO DISABLE THIS!!!)

        // NOTE: FCL works so that a single CollisionData object could be used to
        // accummulate the results from multiple collision detection tasks!!!
        fcl::test::CollisionData<double> self_collision_data;
        self_collision_data.request.enable_contact = true;
        self_collision_data.request.num_max_contacts = 10'000;

        fcl::test::CollisionData<double> scene_collision_data;
        scene_collision_data.request.enable_contact = true;
        scene_collision_data.request.num_max_contacts = 10'000;

        // TEST(To separate in another class(must not be done for self-collision detection???))
        fcl::test::DistanceData<double> scene_distance_data;
        scene_distance_data.request.enable_nearest_points = true;

        auto robot_collision_manager = _robot_collision_model->get_broadphase_managers();

        cout << "Number of managed robot collision meshes" << robot_collision_manager->size() << endl;
        cout << "Number of managed scene collision meshes" << _scene_collision_manager->size() << endl;

        robot_collision_manager->collide(
            robot_collision_manager,
            &self_collision_data,
            CollisionModel::self_collision_function<double>
            //fcl::test::defaultCollisionFunction<double>
        );

        cout << "I CRASH AFTER THIS" << endl;

        robot_collision_manager->collide(
            _scene_collision_manager,
            &scene_collision_data,
            CollisionModel::scene_collision_function<double> // FOR LOGGING!!!
            //fcl::test::defaultCollisionFunction<double>
        );

        cout << "I CRASH AFTER THIS" << endl;

        // TEST(To separate in another class(must not be done for self-collision detection???))
        robot_collision_manager->distance(
            _scene_collision_manager,
            &scene_distance_data,
            //CollisionModel::scene_collision_function<double> // FOR LOGGING!!!
            fcl::test::defaultDistanceFunction<double>
        );

        cout << "I CRASH AFTER THIS" << endl;

        cout << endl;
        cout << "Self-collision points: " << self_collision_data.result.numContacts() << endl;
        cout << "Robot-scene points: " << scene_collision_data.result.numContacts() << endl;
        cout << "Min-distance from scene: " << scene_distance_data.result.min_distance << endl; // TEST
        cout << endl;

        // Contact points aggregation
        points_array_t contact_points;

        // TEST(add to result contacts)
        contact_points.push_back(
            {{
                scene_distance_data.result.nearest_points[0].x(),
                scene_distance_data.result.nearest_points[0].y(),
                scene_distance_data.result.nearest_points[0].z()
            }}
        );

        contact_points.push_back(
            {{
                scene_distance_data.result.nearest_points[1].x(),
                scene_distance_data.result.nearest_points[1].y(),
                scene_distance_data.result.nearest_points[1].z()
            }}
        );

        // SAMPLING_BOUNDS_DEBUG_OUTPUT
        // TEST - ORIGINAL
        // {{ 0.6, 0, 0 }},
        // {{ -0.6, 0.6, 0.6 }},
        // TEST - TRANSFORMED FOR OGRE3D COORDINATE SYSTEM
        // {{ -0.6, 0, 0 }},
        // {{ 0.6, 0.6, 0.6 }},
        // contact_points.push_back(
        //     {{ -0.6, 0, 0 }}
        // );
        // contact_points.push_back(
        //     {{ 0.6, 0.6, 0.6 }}
        // );

        // Add self-collision contact points
        for(size_t i = 0; i < self_collision_data.result.numContacts(); i++) {
            contact_points.push_back(
                {{
                    self_collision_data.result.getContact(i).pos.x(),
                    self_collision_data.result.getContact(i).pos.y(),
                    self_collision_data.result.getContact(i).pos.z()
                }}
            );
        }

        // Add robot-scene collision contact points
        for(size_t i = 0; i < scene_collision_data.result.numContacts(); i++) {
            contact_points.push_back(
                {{
                    scene_collision_data.result.getContact(i).pos.x(),
                    scene_collision_data.result.getContact(i).pos.y(),
                    scene_collision_data.result.getContact(i).pos.z()
                }}
            );
        }

        // Show contact points
        _point_cloud_view->set_points(contact_points);
    }
}