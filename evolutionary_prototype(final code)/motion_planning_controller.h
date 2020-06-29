#pragma once

#include <Ogre.h>
#include <OgreApplicationContext.h>

#include "cartesian_controller.h"
#include "motion_planner.h"
#include "point_cloud_view.h"
#include "graph_view.h"
#include "path_view.h"
#include "trajectory_controller.h"
#include "input_manager.h"
#include "util.h"

class MotionPlanningController : public OgreBites::InputListener {
private:
    CartesianController*  _cartesian_controller  { nullptr };
    MotionPlanner*        _motion_planner        { nullptr };
    PointCloudView*       _point_cloud_view      { nullptr };
    GraphView*            _graph_edges_view      { nullptr };
    PathView*             _path_view             { nullptr };
    TrajectoryController* _trajectory_controller { nullptr };
    InputManager*         _input_manager         { nullptr };

    config_space_index_t _initial_state_index { -1 };
    config_space_index_t _final_state_index   { -1 };

    void _update_graph_view();

public:
    MotionPlanningController(
        CartesianController* cartesian_controller,
        MotionPlanner* motion_planner,
        PointCloudView* point_cloud_view,
        GraphView* graph_edges_view,
        PathView* path_view,
        TrajectoryController* trajectory_controller,
        InputManager* input_manager
    );
    
    // Toggle the different axes for position/orientation
	bool keyPressed(const OgreBites::KeyboardEvent& evt) override;
};
