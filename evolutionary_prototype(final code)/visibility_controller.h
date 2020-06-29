#pragma once

#include <array>

#include <Ogre.h>
#include <OgreApplicationContext.h>

#include "collision_view.h"
#include "input_manager.h"
#include "graph_view.h"
#include "point_cloud_view.h"
#include "util.h"

class VisibilityController : public OgreBites::InputListener {
private:
    CollisionView*       _collision_view            { nullptr };
    GraphView*           _graph_view                { nullptr };
    InputManager*        _input_manager             { nullptr };
    PointCloudView*      _point_cloud_view          { nullptr };

    bool                 _is_collision_view_visible  { true };
    bool                 _is_graph_view_visible      { true };
    bool                 _is_point_cloud_visible     { true };

public:
    VisibilityController(
        CollisionView* collision_view,
        GraphView* graph_view,
        PointCloudView* point_cloud_view,
        InputManager* input_manager
    );
    
    // Toggle visibility for the different objects
	bool keyPressed(const OgreBites::KeyboardEvent& evt) override;
};
