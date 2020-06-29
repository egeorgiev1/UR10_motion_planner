#include "visibility_controller.h"

#include <vector>
#include <iostream>

using namespace std;

VisibilityController::VisibilityController(
    CollisionView* collision_view,
    GraphView* graph_view,
    PointCloudView* point_cloud_view,
    InputManager* input_manager
) :
    _collision_view(collision_view),
    _graph_view(graph_view),
    _point_cloud_view(point_cloud_view),
    _input_manager(input_manager)
{
    _input_manager->add_listener(this);
}

// Toggle the different axes for position/orientation
bool VisibilityController::keyPressed(const OgreBites::KeyboardEvent& evt) {
    switch(evt.keysym.sym) {
        case 'c': {
            _is_collision_view_visible = !_is_collision_view_visible;
            _collision_view->set_visibility(_is_collision_view_visible);
            break;
        }
        case 'v': {
            _is_graph_view_visible = !_is_graph_view_visible;
            _graph_view->set_visibility(_is_graph_view_visible);
            break;
        }
        case 'b': {
            _is_point_cloud_visible = !_is_point_cloud_visible;
            _point_cloud_view->set_visibility(_is_point_cloud_visible);
            break;
        }
        default: {
            break;
        }
    }
}
