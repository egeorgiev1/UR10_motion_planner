#pragma once

#include <array>
#include <vector>
#include <map>

#include "fcl/fcl.h"

#include "forward_kinematics.h"
#include "inverse_kinematics.h"
#include "i_nearest_neighbour.h"
#include "collision_model.h"
#include "util.h"

struct ConfigSpacePoint {
    std::array<real_t, 6> joint_angles          {{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 }};
    std::array<real_t, 3> position              {{ 0.0, 0.0, 0.0 }};
    std::array<real_t, 4> orientation           {{ 0.0, 0.0, 0.0, 0.0 }}; // quaternion
    real_t                min_obstacle_distance { 0.0 };
};

typedef std::vector<ConfigSpacePoint> trajectory_t;

struct RoadmapEdge {
    intmax_t     start_index      { 0 };
    intmax_t     end_index        { 0 };
    trajectory_t local_trajectory;
};

typedef intmax_t config_space_index_t;
typedef intmax_t edge_index_t;

typedef std::vector<edge_index_t> path_t; // used as input and output for the path optimization step
typedef std::vector<RoadmapEdge> standalone_path_t;

// RoadmapEdge da ne e sushto index na samia edge, zashtoto triabva da go reusevam nalyavo i nadiasno(triabva
// da dobavia samia edge kam samia destination i origin, zashtoto e neorientiran graph!!! dvuposochno mozesh
// da se dvizish!!! )
// TODO: Change from a map to a vector???
typedef std::map<config_space_index_t, vector<edge_index_t>> roadmap_adjacency_list_t; // associates the ConfigSpacePoint index to RoadmapEdges to other ConfigSpacePoints

class MotionPlanner {
private:
    CollisionModel*                          _robot_collision_model   { nullptr };
    fcl::BroadPhaseCollisionManager<real_t>* _scene_collision_manager { nullptr };
    ForwardKinematics*                       _forward_kinematics      { nullptr };
    InverseKinematics*                       _inverse_kinematics      { nullptr };
    INearestNeighbour*                       _nearest_neighbour       { nullptr };

    std::default_random_engine               _generator; // NOTE: Seed is NOT set to a random number automatically!!!

    intmax_t _start_index { -1 }; // Config space index
    intmax_t _end_index   { -1 }; // Config space index

    std::vector<ConfigSpacePoint>            _config_space;
    std::vector<RoadmapEdge>                 _roadmap_edges;
    roadmap_adjacency_list_t                 _roadmap_adjacency_list;

    std::array<std::array<real_t, 3>, 2>     _bounds {{
        {{ 0.0, 0.0, 0.0 }},
        {{ 0.0, 0.0, 0.0 }}
    }};
    intmax_t                                 _sample_count               { 0 };
    intmax_t                                 _ik_sample_count            { 0 };
    real_t                                   _step_size                  { 0.0 };
    real_t                                   _nearest_neighbour_radius   { 0.0 };
    real_t                                   _min_distance_from_obstacle { 0.0 };
    real_t                                   _max_error                  { 0.0 };

public:
    MotionPlanner(
        CollisionModel*                          robot_collision_model,
        fcl::BroadPhaseCollisionManager<real_t>* scene_collision_manager,
        ForwardKinematics*                       forward_kinematics,
        InverseKinematics*                       inverse_kinematics,
        INearestNeighbour*                       nearest_neighbour,
        std::array<std::array<real_t, 3>, 2>     bounds, // Cuboid defined by two points(the cuboid's diagonal)
        intmax_t                                 sample_count,
        intmax_t                                 ik_sample_count,
        real_t                                   step_size,
        real_t                                   nearest_neighbour_radius,
        real_t                                   min_distance_from_obstacle,
        real_t                                   max_error
    );

    // TODO: calculate with KDL end-effector etc., also min_distance!!!
    intmax_t set_initial_state(std::array<real_t, 6> joint_angles);
	intmax_t set_final_state(std::array<real_t, 6> joint_angles);

    /** Utility methods */
    bool _are_joint_angles_equal(
        std::array<real_t, 6> j1,
        std::array<real_t, 6> j2
    );

    std::array<real_t, 6> _generate_random_angles();

    vector<ConfigSpacePoint> _sample_ik_solutions(
        std::array<real_t, 3> position,
        std::array<real_t, 4> orientation
    );

    // Generates ConfigSpacePoints(using ik_sample) and calls _index
    std::vector<intmax_t> _add_by_tcp(
        std::array<real_t, 3> position,
        std::array<real_t, 4> orientation
    );

    // Generates ConfigSpacePoint and calls _index
    intmax_t _add_by_joint_angles(std::array<real_t, 6> joint_angles);

    /** Main motion planning methods */
    intmax_t _index_point(ConfigSpacePoint point);

    void _sample_config_space();

    path_t _shortest_path();

    standalone_path_t _path_optimizer(path_t path);

    trajectory_t _standalone_path_to_trajectory(standalone_path_t path);

    trajectory_t _path_to_trajectory(path_t path);

    /** Getters **/
    const std::vector<ConfigSpacePoint>& get_config_space();

    const std::vector<RoadmapEdge>& get_roadmap_edges();

    const roadmap_adjacency_list_t& get_roadmap_adjacency_list();

    /**
     * @brief Produces a trajectory from `start` to `end`, in which the robot moves
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
    intmax_t _local_planner( // TOOD: Better name like "_plan_locally" ???
        ConfigSpacePoint start, // TODO: use indexes here instead???
        ConfigSpacePoint end, // TODO: use indexes here instead???
        //real_t min_distance_from_obstacle,
        //intmax_t step_size,

        // Output
        trajectory_t& output_trajectory
    );

    void _set_collision_model_angles(std::array<real_t, 6> joint_angles);
    
    /**
     * @brief Self-collision check for the given joint angles.
     */
    bool _is_self_colliding();

    // Also used for collision detection with the scene(is -1 when the robot collides with the scene)
    real_t _scene_min_distance();
};

