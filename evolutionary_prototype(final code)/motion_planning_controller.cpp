#include "motion_planning_controller.h"

#include <array>
#include <vector>
#include <iostream>

using namespace std;

MotionPlanningController::MotionPlanningController(
    CartesianController* cartesian_controller,
        MotionPlanner* motion_planner,
        PointCloudView* point_cloud_view,
        GraphView* graph_edges_view,
        PathView* path_view,
        TrajectoryController* trajectory_controller,
        InputManager* input_manager
) :
    _cartesian_controller(cartesian_controller),
    _motion_planner(motion_planner),
    _point_cloud_view(point_cloud_view),
    _graph_edges_view(graph_edges_view),
    _path_view(path_view),
    _trajectory_controller(trajectory_controller),
    _input_manager(input_manager)
{
    _input_manager->add_listener(this);
}

void MotionPlanningController::_update_graph_view() {
    points_array_t vertices;
    std::vector<GraphEdge> graph_edges;

    const auto& config_space = _motion_planner->get_config_space();
    const auto& roadmap_edges = _motion_planner->get_roadmap_edges();

    for(const RoadmapEdge& edge : roadmap_edges) {
        graph_edges.push_back({
            {{ 
                -config_space[edge.start_index].position[0],
                config_space[edge.start_index].position[2],
                config_space[edge.start_index].position[1],
            }},
            {{  
                -config_space[edge.end_index].position[0],
                config_space[edge.end_index].position[2],
                config_space[edge.end_index].position[1],
            }}
        });

        vertices.push_back(
            {{
                -config_space[edge.start_index].position[0],
                config_space[edge.start_index].position[2],
                config_space[edge.start_index].position[1],
            }}
        );

        vertices.push_back(
            {{
                -config_space[edge.end_index].position[0],
                config_space[edge.end_index].position[2],
                config_space[edge.end_index].position[1],
            }}
        );
    }

    _point_cloud_view->set_points(vertices);
    _graph_edges_view->set_edges(graph_edges);
}

bool MotionPlanningController::keyPressed(const OgreBites::KeyboardEvent& evt) {
    switch(evt.keysym.sym) {
        case 'n': {
            _initial_state_index = _motion_planner->set_initial_state(
                _cartesian_controller->get_angles()
            );

            if(_initial_state_index == -1) {
                cout << "Invalid initial state" << endl;
            } else {
                _update_graph_view();
            }

            break;
        }
        case 'm': {
            _final_state_index = _motion_planner->set_final_state(
                _cartesian_controller->get_angles()
            );

            if(_final_state_index == -1) {
                cout << "Invalid final state" << endl;
            } else {
                _update_graph_view();
            }
            break;
        }
        case 'o': {
            cout << "Sampling, this might take a while..." << endl;
            _motion_planner->_sample_config_space();

            _update_graph_view();
            break;
        }
        case 'p': {
            cout << "Calculating shortest path, this might take a while..." << endl;

            if(_initial_state_index != -1 && _final_state_index != -1) {

                const auto& config_space = _motion_planner->get_config_space();
                const auto& roadmap_edges = _motion_planner->get_roadmap_edges();

                path_t shortest_path = _motion_planner->_shortest_path();

                standalone_path_t optimized_path = _motion_planner->_path_optimizer(shortest_path);
                cout << "Shortest path length: " << shortest_path.size() << endl;
                cout << "Optimized path length: " << optimized_path.size() << endl;

                if(shortest_path.size()) {

                    trajectory_t shortest_path_trajectory =
                        _motion_planner->_standalone_path_to_trajectory(
                            optimized_path
                        )
                    ;

                    // Visualize path and enable trajectory control
                    std::vector<std::array<real_t, 3>> shortest_path_points;

                    for(const auto& edge : optimized_path) {
                        shortest_path_points.push_back(
                            {{
                                -config_space[edge.start_index].position[0],
                                config_space[edge.start_index].position[2],
                                config_space[edge.start_index].position[1]
                            }}
                        );
                    }

                    // Add final point from the shortest_path
                    shortest_path_points.push_back(
                        {{
                            -config_space[(*optimized_path.rbegin()).end_index].position[0],
                            config_space[(*optimized_path.rbegin()).end_index].position[2],
                            config_space[(*optimized_path.rbegin()).end_index].position[1]
                        }}
                    );

                    _path_view->set_points(shortest_path_points);
                    _trajectory_controller->set_trajectory(shortest_path_trajectory);
                } else {
                    cout << "No valid path found from initial to final state." << endl;
                }
            }
            break;
        }
        default: {
            break;
        }
    }
}
