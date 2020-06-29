#pragma once

#include <Ogre.h>
#include <OgreApplicationContext.h>
#include <fcl/fcl.h>

#include "point_cloud_view.h"
#include "collision_model.h"

class CollisionController : public OgreBites::InputListener {
private:
    CollisionModel*                          _robot_collision_model   { nullptr };
    PointCloudView*                          _point_cloud_view        { nullptr };
    fcl::BroadPhaseCollisionManager<double>* _scene_collision_manager { nullptr };

    bool _must_perform_collision_detection { false };
public:
    CollisionController(
        CollisionModel* robot_collision_model,
        PointCloudView* point_cloud_view,
        fcl::BroadPhaseCollisionManager<double>* scene_collision_manager
    );

    void schedule_collision_detection();

	void frameRendered(const Ogre::FrameEvent& evt) override;
};