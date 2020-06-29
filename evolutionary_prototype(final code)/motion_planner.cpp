#include "motion_planner.h"

#include <cmath>
#include <iostream>
#include <algorithm>
#include <random>

#include <kdl/chainfksolverpos_recursive.hpp>

// DEBUG
//#define LOGGING_ENABLED
//#define LOGGING_SHORTEST_PATH_ENABLED

void log(string to_log) {
    #ifdef LOGGING_ENABLED
        using namespace std;
        std::cout << to_log;
    #endif
}


void log_shortest_path(string to_log) {
    #ifdef LOGGING_SHORTEST_PATH_ENABLED
        using namespace std;
        std::cout << to_log;
    #endif
}

MotionPlanner::MotionPlanner(
    CollisionModel*                          robot_collision_model,
    fcl::BroadPhaseCollisionManager<real_t>* scene_collision_manager,
    ForwardKinematics*                       forward_kinematics,
    InverseKinematics*                       inverse_kinematics,
    INearestNeighbour*                       nearest_neighbour,
    std::array<std::array<real_t, 3>, 2>     bounds,
    intmax_t                                 ik_sample_count,
    intmax_t                                 sample_count,
    real_t                                   step_size,
    real_t                                   nearest_neighbour_radius,
    real_t                                   min_distance_from_obstacle,
    real_t                                   max_error
) :
    _robot_collision_model(robot_collision_model),
    _scene_collision_manager(scene_collision_manager),
    _forward_kinematics(forward_kinematics),
    _inverse_kinematics(inverse_kinematics),
    _nearest_neighbour(nearest_neighbour),
    _bounds(bounds),
    _ik_sample_count(ik_sample_count),
    _sample_count(sample_count),
    _step_size(step_size),
    _nearest_neighbour_radius(nearest_neighbour_radius),
    _min_distance_from_obstacle(min_distance_from_obstacle),
    _max_error(max_error)
{}

// RETURN -1 ON ERROR, return the ConfigSpacePoint index otherwise
intmax_t MotionPlanner::set_initial_state(std::array<real_t, 6> joint_angles) {

    _start_index = _add_by_joint_angles(joint_angles); // Can return -1

    // Index point (adds it to the roadmap)
    return _start_index;
}


// RETURN -1 ON ERROR, return the ConfigSpacePoint index otherwise
intmax_t MotionPlanner::set_final_state(std::array<real_t, 6> joint_angles) {
    
    _end_index = _add_by_joint_angles(joint_angles); // Can return -1

    // Index point (adds it to the roadmap)
    return _end_index;
}

// Generates ConfigSpacePoints(using ik_sample) and calls _index
//
// NOTE: Can return an empty array
std::vector<intmax_t> MotionPlanner::_add_by_tcp(
    std::array<real_t, 3> position,
    std::array<real_t, 4> orientation
) {
    std::vector<intmax_t> config_point_indexes;

    // Sample IK solutions
    log("Sampling IK solutions for given TCP...\n");
    auto ik_solutions = _sample_ik_solutions(
        position,
        orientation
    );
    log("Number of IK solutions found: "s + to_string(ik_solutions.size()) + "\n"s);

    // Generate configuration space points from the IK solutions
    // and index them
    for(const auto& ik_solution : ik_solutions) {
        intmax_t index = _index_point(ik_solution);

        if(index != -1) {
            log("Successfully indexed\n");
            config_point_indexes.push_back(index);
        } else {
            log("Unsuccessful indexing\n");
        }
    }

    return config_point_indexes;
}

// Generates ConfigSpacePoint and calls _index
intmax_t MotionPlanner::_add_by_joint_angles(std::array<real_t, 6> joint_angles) {
    ConfigSpacePoint final_state_point; // TODO: rename to config_state_point instead

    // Use KDL to compute end effector position orientation, because ForwardKinematics
    // gives me an end effector's position and orientation in Ogre's coordinate system

    KDL::Chain chain = InverseKinematics::_joint_frames(); // Must be done like this, otherwise
    // kdl_fk_solver won't work, probably because the constructor takes a reference and internally
    // stores a pointer to the chain?
    KDL::ChainFkSolverPos* _kdl_fk_solver = new  KDL::ChainFkSolverPos_recursive {
        chain
    };

    KDL::JntArray input_angles(6);
    
    // Initialize input array of angles
    for(intmax_t i = 0; i < 6; i++) {
        input_angles(i) = joint_angles[i];
    }
    
    // Create destination frame
    KDL::Frame result_frame;

    _kdl_fk_solver->JntToCart(input_angles, result_frame);

    // Compute TCP using ForwardKinematics
    _forward_kinematics->set_angles(joint_angles);

    // final_state_point.joint_angles = joint_angles;

    // final_state_point.position = 
    //     _forward_kinematics->get_tcp_position(); // TODO: Maybe it needs to be transformed?!

    // final_state_point.orientation = 
    //     _forward_kinematics->get_tcp_orientation();

    final_state_point.position = {{
        result_frame.p[0],
        result_frame.p[1],
        result_frame.p[2],
    }};

    real_t w, x, y, z;
    result_frame.M.GetQuaternion(w, x, y, z);

    final_state_point.orientation = {{
        w, x ,y, z
    }};

    // final_state_point.orientation = 
    //     _forward_kinematics->get_tcp_orientation();

    // TO REMOVE
    // _inverse_kinematics->set_target_position(final_state_point.position);
    _inverse_kinematics->set_target_position({{
        final_state_point.position[0],
        final_state_point.position[1],
        final_state_point.position[2]
    }});
    _inverse_kinematics->set_target_orientation(final_state_point.orientation);
    _inverse_kinematics->set_current_angles(joint_angles);

    std::array<real_t, 6> output_joint_angles { 0, 0, 0, 0, 0, 0 };
    intmax_t error = _inverse_kinematics->solve(output_joint_angles);

    bool ans = _are_joint_angles_equal(joint_angles, output_joint_angles);
    
    // log_shortest_path("Before alteration:");
    // log_shortest_path("\n");
    // for(intmax_t i = 0; i < 6; i++) {
    //     log_shortest_path(to_string(joint_angles[i]) + ", ");
    // }
    // log_shortest_path("\n");
    // log_shortest_path("After alteration:");
    // log_shortest_path("\n");
    // for(intmax_t i = 0; i < 6; i++) {
    //     log_shortest_path(to_string(output_joint_angles[i]) + ", ");
    // }
    // log_shortest_path("\n");
    // log_shortest_path(
    //     "After orientation: "s +
    //     to_string(final_state_point.orientation[0]) + ", "s +
    //     to_string(final_state_point.orientation[1]) + ", "s +
    //     to_string(final_state_point.orientation[2]) + ", "s +
    //     to_string(final_state_point.orientation[3])
    // );
    // log_shortest_path("\n");

    // log_shortest_path(
    //     "After position: "s +
    //     to_string(final_state_point.position[0]) + ", "s +
    //     to_string(final_state_point.position[1]) + ", "s +
    //     to_string(final_state_point.position[2])
    // );
    // log_shortest_path("\n");

    final_state_point.joint_angles = output_joint_angles;

    delete _kdl_fk_solver;
    return _index_point(final_state_point); // Can return -1
}

// Returns the index in the vertices array. Returns -1 if collision and proximity checks failed.
// NOTE: Assumes joint angles correspond to the TCP in ConfigSpacePoint
// Does collision and proximity checks. If passed, adds config space point
// to nearest neighbour index and to the roadmap graph
intmax_t MotionPlanner::_index_point(ConfigSpacePoint point) { // NOTE: this will be called when adding an initial and end point!!!

    // Evaluate self-collision, collision with the scene and proximity constraint
    _set_collision_model_angles(point.joint_angles);

    // If not self-colliding
    if(!_is_self_colliding()) {
        real_t min_distance = _scene_min_distance();
        
        // If min-distance from scene geometry constraint satisfied
        if(_min_distance_from_obstacle <= min_distance) {
            point.min_obstacle_distance = min_distance;
        } else {
            log("Min-obstacle constraint violated or scene collision has occurred in _index_point\n");
            return -1;
        }
    } else {
        log("Self-colliding error in _index_point\n");
        return -1;
    }

	// Add to the graph vertices
    _config_space.push_back(point);

	// Add to the nearest neighbour index
    _nearest_neighbour->addPoint(
        _config_space.size() - 1, // index
        point.position            // TCP position
    );

	// Query nearest neighbours
     std::vector<intmax_t> neighbours =
        _nearest_neighbour->getInRadius(
             point.position,
            _nearest_neighbour_radius
        )
    ;

	// Perform local planning for all neighbours. If local planning is successful,
    // generate two edges for both directions of travel along the edge.
    for(intmax_t neighbour_index : neighbours) {
        
        // Exclude edge from current point to itself
        if(neighbour_index != _config_space.size() - 1) {

            trajectory_t local_trajectory;

            // Doesn't matter which is the start and which is the end.
            // It must work both ways.
            intmax_t local_planner_error =
                _local_planner (
                    _config_space[_config_space.size() - 1],
                    _config_space[neighbour_index],
                    local_trajectory
                )
            ;

            // If local planning is successful then add an edge in
            // both directions to the adjacency_list
            if(!local_planner_error) {
                _roadmap_edges.push_back(
                    RoadmapEdge {
                        _config_space.size() - 1,
                        neighbour_index,
                        local_trajectory   
                    }
                );

                _roadmap_adjacency_list[_config_space.size() - 1].push_back(
                    _roadmap_edges.size() - 1
                );

                // We need to reverse the trajectory, but then the set of
                // points in the trajectory will be like [start; end), but
                // it should be like (start, end]. This must be corrected!
                trajectory_t reversed_trajectory = local_trajectory;

                reversed_trajectory.pop_back();

                std::reverse(
                    reversed_trajectory.begin(),
                    reversed_trajectory.end()
                );

                reversed_trajectory.push_back(
                    _config_space[_config_space.size() - 1]
                );

                _roadmap_edges.push_back(
                    RoadmapEdge {
                        neighbour_index,
                        _config_space.size() - 1,
                        reversed_trajectory
                    }
                );

                _roadmap_adjacency_list[neighbour_index].push_back(
                    _roadmap_edges.size() - 1
                );
            }
        }
    }

    // Return index of the given ConfigSpacePoint
    return _config_space.size() - 1;
}

bool MotionPlanner::_are_joint_angles_equal(
    std::array<real_t, 6> j1,
    std::array<real_t, 6> j2
) {
    bool are_equal = true;
    
    for(intmax_t i = 0; i < 6; i++) {
        // Handle unnormalized angles(wrap-around)
        real_t delta = atan2(sin(j1[i] - j2[i]), cos(j1[i] - j2[i]));

        if(fabs(delta) > _max_error) {
            are_equal = false;
            break;
        }
    }

    return are_equal;
}

std::array<real_t, 6> MotionPlanner::_generate_random_angles() {
    std::array<real_t, 6> generated_angles;

    std::uniform_real_distribution<real_t> distribution(0, 2*M_PI);

    for(intmax_t i = 0; i < 6; i++) {    
        generated_angles[i] = distribution(_generator);
    }

    return generated_angles;
}

// Can return an empty vector if no IK solution is found
vector<ConfigSpacePoint> MotionPlanner::_sample_ik_solutions(
    std::array<real_t, 3> position,
    std::array<real_t, 4> orientation
) {
    std::vector<std::array<real_t, 6>> ik_solutions;
    
    for(intmax_t i = 0; i < _ik_sample_count; i++) {
         // Solve IK for random initial joint angles
         std::array<real_t, 6> random_angles {
             _generate_random_angles()
         };

        _inverse_kinematics->set_target_position(position);
        _inverse_kinematics->set_target_orientation(orientation);
        _inverse_kinematics->set_current_angles(random_angles);

        std::array<real_t, 6> output_joint_angles { 0, 0, 0, 0, 0, 0 };
        intmax_t error = _inverse_kinematics->solve(output_joint_angles);

        if(error != -1) {
            // log("Joint angles solution: ");

            // for(intmax_t i = 0; i < 6; i++) {
            //     log(to_string(output_joint_angles[i]) + ", "s);
            // }

            // log("\n");

            // // log("Random angles: ");

            // // for(intmax_t i = 0; i < 6; i++) {
            // //     log(to_string(random_angles[i]) + ", "s);
            // // }

            // log("\n");
        }

        // If valid IK solution
        if(error != -1) {
            // Check if duplicate IK solution
            bool is_duplicate = false;

            for(const auto& ik_solution: ik_solutions) {
                if(_are_joint_angles_equal(ik_solution, output_joint_angles)) {
                    is_duplicate = true;
                    break;
                }
            }

            // Add to IK solutions if not duplicate
            if(!is_duplicate) {
                log("Joint angles solution: ");

                for(intmax_t i = 0; i < 6; i++) {
                    log(to_string(output_joint_angles[i]) + ", "s);
                }

                log("\n");

                ik_solutions.push_back(output_joint_angles);
            }
        }
    }

    // Adapt to output format
    vector<ConfigSpacePoint> result;

    for(const auto& ik_solution: ik_solutions) {
        result.push_back(
            ConfigSpacePoint {
                ik_solution,
                position,
                orientation,
                0.0
            }
        );
    }

    return result;
}

// Uniform sampling (Use Eigen for quaternion uniform sampling)
void MotionPlanner::_sample_config_space() {

    // Setup random position generators and distributions in the cuboid's bounds
    std::uniform_real_distribution<real_t> distribution_x {
        _bounds[0][0],
        _bounds[1][0]
    };

    std::uniform_real_distribution<real_t> distribution_y {
        _bounds[0][1],
        _bounds[1][1]
    };

    std::uniform_real_distribution<real_t> distribution_z {
        _bounds[0][2],
        _bounds[1][2]
    };

    // Generate _sample_count number of samples
    for(intmax_t i = 0; i < _sample_count; i++) {
        cout << ("Current sample number: "s + to_string(i) + "\n");
        log("Current sample number: "s + to_string(i) + "\n");

        // Generate random position
        std::array<real_t, 3> random_position {{
            distribution_x(_generator),
            distribution_y(_generator),
            distribution_z(_generator)
        }};

        // Generate random orientation (uniform distribution)
        fcl::Quaterniond random_orientation_quaternion = fcl::Quaterniond::UnitRandom();

        std::array<real_t, 4> random_orientation {{
            random_orientation_quaternion.w(),
            random_orientation_quaternion.x(),
            random_orientation_quaternion.y(),
            random_orientation_quaternion.z()
        }};

        // Sample IK and index the resulting config space points
        _add_by_tcp(
            random_position,
            random_orientation
        );
    }
}

// Dijkstra from "start" to "end". It will return an empty path
// if no path can be found
path_t MotionPlanner::_shortest_path() {

    log_shortest_path("CONFIG SPACE SIZE: "s + to_string(_config_space.size()) + "\n"s);

    std::vector<bool> is_processed (
        _config_space.size(),
        false
    );

    std::vector<intmax_t> previous_edge_index (
        _config_space.size(),
        -1
    );

    std::vector<real_t> distance (
        _config_space.size(),
        std::numeric_limits<real_t>::infinity()
    );

    std::set<std::pair<real_t, intmax_t>> queue;

    distance[_start_index] = 0;

    // Populate queue with all vertices
    for(intmax_t i = 0; i < _config_space.size(); i++) {
        queue.insert(
            std::make_pair(distance[i], i)
        );
    }

    // for(const auto& neighbours : _roadmap_adjacency_list) {
    //     log_shortest_path(
    //         "NUMBER OF NEIGHBOURS "s + to_string(neighbours.first) + ": "s +
    //         to_string(neighbours.second.size()) + "\n"s
    //     );

    //     log_shortest_path(
    //         "NUMBER OF NEIGHBOURS: "s + to_string(_roadmap_adjacency_list.at(3).size()) + "\n"s
    //     );
    // }

    // for(const auto& neighbours : _roadmap_adjacency_list) {
    //     log_shortest_path(
    //         "NUMBER OF NEIGHBOURS "s + to_string(neighbours.first) + ": "s +
    //         to_string(neighbours.second.size()) + "\n"s
    //     );

    //     log_shortest_path(
    //         "NUMBER OF NEIGHBOURS: "s + to_string(_roadmap_adjacency_list[0].size()) + "\n"s
    //     );
    // }

    // Main Dijkstra loop
    while(!queue.empty()) {

        // log_shortest_path(
        //     "NUMBER OF NEIGHBOURS: "s + to_string(_roadmap_adjacency_list[0].size()) + "\n"s
        // );

        // log_shortest_path(
        //     "NUMBER OF NEIGHBOURS: "s + to_string(_roadmap_adjacency_list[1].size()) + "\n"s
        // );

        // Get top element
        std::pair<real_t, intmax_t> top_node = *queue.begin();

        log_shortest_path("NODE PROCESSED: "s + to_string(top_node.second) + "\n"s);

        // Remove top element from queue
        queue.erase(
            queue.begin()
        );

        // If the top of the queue is the final node, break
        if(top_node.second == _end_index) {
            log_shortest_path("FOUND SHORTEST PATH!!!\n"s);
            break;
        }

        is_processed[top_node.second] = true;

        // For all neighbours of top_node, try improving their distance. if improved,
        // update previous[], distance[] and queue    
        for(edge_index_t edge_index : _roadmap_adjacency_list[top_node.second]) {

            // Compute edge distance
            intmax_t edge_start_index = _roadmap_edges[edge_index].start_index;
            intmax_t edge_end_index = _roadmap_edges[edge_index].end_index;

            real_t edge_distance = euclidean_distance(
                _config_space[edge_start_index].position,
                _config_space[edge_end_index].position
            );
            
            // Try improving the neighbour's distance
            real_t new_distance = distance[top_node.second] + edge_distance;

            log_shortest_path("new dist "s + to_string(edge_index) + ": " + to_string(new_distance) + "\n"s);

            // What if the element was already processed??? If it was then it
            // won't be able to improve anymore. It's optimum was already found.
            // We don't need to have a is_processed check for entering this if statement???
            if(new_distance < distance[edge_end_index] && is_processed[edge_end_index] == false) {
                log_shortest_path("Updating distance of " + to_string(edge_end_index) + "\n"s);

                // Remove outdated queue entry
                queue.erase(
                    make_pair(distance[edge_end_index], edge_end_index)
                ); 

                distance[edge_end_index] = new_distance;
                previous_edge_index[edge_end_index] = edge_index;

                // Reinsert into the queue
                queue.insert(
                    make_pair(new_distance, edge_end_index)
                );   
            }
        }
    }

    static intmax_t i = 0;
    for(auto dist : distance) {
        log_shortest_path("dist "s + to_string(i++) + ": " + to_string(dist) + "\n"s);
    }

    path_t shortest_path;

    // Reconstruct shortest path
    intmax_t current_edge = previous_edge_index[_end_index];

    while(current_edge != -1){
        shortest_path.push_back(current_edge);

        current_edge = previous_edge_index[
            _roadmap_edges[current_edge].start_index
        ];
    }

    // Reverse path
    std::reverse(shortest_path.begin(), shortest_path.end());

    return shortest_path;
}

standalone_path_t MotionPlanner::_path_optimizer(path_t path) {
    
    standalone_path_t optimized_path;

    for(edge_index_t edge_index : path) {
        optimized_path.push_back(
            _roadmap_edges[edge_index]
        );

        // cout << "EDGE INDEX: " << edge_index << endl;
    }

    // Works by trying to reduce 2 subsequent edges into one edge by taking a direct
    // path from the beginning of the first edge to the end of the second edge.

    intmax_t e1 = 0;
    intmax_t e2 = 1;
    while(e2 < optimized_path.size()) {
        
        trajectory_t local_trajectory;

        config_space_index_t start_index = optimized_path[e1].start_index;
        config_space_index_t end_index = optimized_path[e2].end_index;

        intmax_t local_planner_error = _local_planner(
            _config_space[start_index],
            _config_space[end_index],
            local_trajectory
        );

        // Replace e1 edge if local trajectory is valid and remove e2
        if(local_planner_error != -1) {
            // cout << "Combining edges: " << e1 << " " << e2 << endl;
            optimized_path[e1] = 
                RoadmapEdge {
                    start_index,
                    end_index,
                    local_trajectory
                }
            ;

            optimized_path.erase(optimized_path.begin() + e2); // Tova mai ne si varshi rabotata kakto triabva???
        } else {
            // cout << "Skipping edges: " << e1 << " " << e2 << endl;

            e1 = e2;
            e2 = e1 + 1;
        }
    }

    return optimized_path;
}

trajectory_t MotionPlanner::_standalone_path_to_trajectory(standalone_path_t path) {

    trajectory_t output_trajectory;

    // Include starting point, because it's not included by default due to
    // the nature of the _local_planner output(see _local_planner documentation)
    output_trajectory.push_back(
        ConfigSpacePoint {
            _config_space[_start_index].joint_angles,
            {{ 0.0, 0.0, 0.0 }},
            {{ 0.0, 0.0, 0.0 }},
            0.0
        }
    );

    for(const RoadmapEdge& edge : path) {
        for(const auto& point : edge.local_trajectory) {
            output_trajectory.push_back(
                ConfigSpacePoint {
                    point.joint_angles,
                    {{ 0.0, 0.0, 0.0 }},
                    {{ 0.0, 0.0, 0.0 }},
                    0.0
                }
            );
        }        
    }

    return output_trajectory;
}

trajectory_t MotionPlanner::_path_to_trajectory(path_t path) {
    
    standalone_path_t optimized_path;

    for(edge_index_t edge_index : path) {
        optimized_path.push_back(
            _roadmap_edges[edge_index]
        );
    }

    return _standalone_path_to_trajectory(optimized_path);
}

const std::vector<ConfigSpacePoint>& MotionPlanner::get_config_space() {
    return _config_space;
}

const std::vector<RoadmapEdge>& MotionPlanner::get_roadmap_edges() {
    return _roadmap_edges;
}

const roadmap_adjacency_list_t& MotionPlanner::get_roadmap_adjacency_list() {
    return _roadmap_adjacency_list;
}

/**
 * @brief Produces a trajectory from `_start` to `_end`, in which the robot moves
 *        in a straight line between these two points.
 * 
 * @returns Returns -1 if one of the following conditions are true:
 * 
 *     1. `min_distance_from_obstacle` constraint cannot be fulfilled for at
 *     least one point in the local trajectory(includes `_start` and `_end`).
 * 
 *     2. Local planner IK does not converge to the joint angles at `end` with
 *     an error specified by `_max_error`.
 * 
 *     3. Self-collision or collision with the scene happens for at least one
 *     point in the local trajectory(includes `_start` and `_end`).
 * 
 *     NOTE: '_start' and '_end' constraints for collision and
 *     `min_distance_from_obstacle` are checked at the start of the function
 *     and if not fulfilled the function terminates early. Can be used for
 *     making a simpler global motion planner implementation.
 * 
 *     NOTE2: Step-size algorithm works by (floor(dist(start, end)/step_size) - 1)
 *     to get the number of points between start and end, which shall be evenly
 *     spaced between `_start` and `_end`.
 */
intmax_t MotionPlanner::_local_planner(
    ConfigSpacePoint start,
    ConfigSpacePoint end,
    //real_t min_distance_from_obstacle,
    //intmax_t step_size,

    // Output
    trajectory_t& output_trajectory
) {
    trajectory_t trajectory; // Excludes 'start', includes 'end'

    // Compute all in-between ConfigSpacePoints
    intmax_t number_of_points_in_between {
        std::llround(
            floor(
                euclidean_distance(start.position, end.position) / _step_size
            ) - 1
        )
    };

    real_t progress_step = 1.0 / (number_of_points_in_between + 1);

    for(intmax_t i = 0; i < number_of_points_in_between; i++) {
        ConfigSpacePoint config_space_point;

        real_t progress = 1.0/(number_of_points_in_between + 2); // between 0 and 1

        // Compute new position(LERP)
        config_space_point.position = {
            lerp(start.position, end.position, progress_step*(i + 1))
        };

        // Compute new orientation(SLERP)
        config_space_point.orientation = {
            slerp(start.orientation, end.orientation, progress_step*(i + 1))
        };

        // Add to the trajectory
        trajectory.push_back(config_space_point);
    }

    // Push end-point
    trajectory.push_back(end);

    // Evaluate IK for all trajectory points
    std::array<real_t, 6> last_joint_angles { start.joint_angles };
    
    for(auto& config_space_point : trajectory) {
        // Calculate joint angles and put them in the trajectory
        _inverse_kinematics->set_target_position(config_space_point.position);
        _inverse_kinematics->set_target_orientation(config_space_point.orientation);
        _inverse_kinematics->set_current_angles(last_joint_angles);

        std::array<real_t, 6> output_joint_angles { 0, 0, 0, 0, 0, 0 };
        intmax_t error = _inverse_kinematics->solve(output_joint_angles);

        if(error == -1) {
            // Kinematically not possible
            log("Kinematically not possible\n");
            return -1;
        }

        // Update last_joint_angles
        last_joint_angles = output_joint_angles;

        // Store resulting joint angles in the trajectory
        config_space_point.joint_angles = output_joint_angles;
    }

    // Check if joint angles of the last point in the trajectory match to the given
    // precision the joint angles in the 'end' variable. If not then this local
    // planning is not possible, because IK must converge to the joint angles at 'end'
    if(!_are_joint_angles_equal(
        trajectory[trajectory.size() - 1].joint_angles,
        end.joint_angles
    )) {
        log("Max joint angles error of last point exceeded\n");
            
        log("End joint angles: ");
        for(intmax_t i = 0; i < 6; i++) {
            log( std::to_string(end.joint_angles[i]) + ", " );
        }
        log("\n");

        log("End joint angles: ");
        for(intmax_t i = 0; i < 6; i++) {
            log( std::to_string(trajectory[trajectory.size() - 1].joint_angles[i]) + ", " );
        }
        log("\n");
        log("\n");

        return -1;
    }

    // Check collision constraints
    // if not fulfilled, return -1
    // if fulfilled, continue
    for(auto& config_space_point : trajectory) { // ADD MIN DISTANCE TO THE CONFIG POINT!!!
        _set_collision_model_angles(config_space_point.joint_angles);

        if(_is_self_colliding()) {
            log("Self-colliding error\n");
            return -1;
        }

        real_t min_distance = _scene_min_distance();

        if(min_distance < 0) {
            log("Colliding with scene error\n");
            return -1;
        } else if(_min_distance_from_obstacle >= min_distance) {
            log("Min obstacle distance not fulfilled\n");
            return -1;
        } else {
            config_space_point.min_obstacle_distance = min_distance;
        }
    }

    // Populate output_trajectory
    output_trajectory = trajectory;

    // Return 0 to indicate successful local planning
    return 0;
}

void MotionPlanner::_set_collision_model_angles(std::array<real_t, 6> joint_angles) {
    _forward_kinematics->set_angles(joint_angles);

    for(intmax_t i = 0; i < 6; i++) {
        _robot_collision_model->set_transform(
            i,
            _forward_kinematics->get_transform(i)
        );
    }
}

/**
 * @brief Self-collision check for the given joint angles.
 */
bool MotionPlanner::_is_self_colliding() {
    fcl::test::CollisionData<double> self_collision_data;
    
    auto robot_collision_manager = _robot_collision_model->get_broadphase_managers();

    robot_collision_manager->collide(
        robot_collision_manager,
        &self_collision_data,
        CollisionModel::self_collision_function<double>
    );

    return self_collision_data.result.isCollision();
}

// Also used for collision detection with the scene(is -1 when the robot collides with the scene)
real_t MotionPlanner::_scene_min_distance() {
    fcl::test::DistanceData<double> scene_distance_data;

    auto robot_collision_manager = _robot_collision_model->get_broadphase_managers();

    robot_collision_manager->distance(
        _scene_collision_manager,
        &scene_distance_data,
        fcl::test::defaultDistanceFunction<double>
    );

    return scene_distance_data.result.min_distance;
}
