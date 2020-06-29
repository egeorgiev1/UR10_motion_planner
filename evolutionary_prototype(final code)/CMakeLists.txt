cmake_minimum_required(VERSION 3.0.0)
project(cmake_project VERSION 0.1.0)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_BUILD_TYPE Release)

# For OgreProcedural
set(CMAKE_MODULE_PATH 
  ${CMAKE_MODULE_PATH}
  ${CMAKE_SOURCE_DIR}/CMake
  ${CMAKE_SOURCE_DIR}/CMake/Utils
  ${CMAKE_SOURCE_DIR}/CMake/Packages
)

# Fix issue where FCL searches for Eigen/Dense, but must be eigen3/Eigen/Dense with my setup
include_directories(/usr/local/include/eigen3)

find_package(Eigen3)
find_package(fcl)

find_package(CCD REQUIRED)
include_directories(${CCD_INCLUDE_DIRS})

find_package(orocos_kdl)
find_package(OGRE 1.11 REQUIRED COMPONENTS Bites RTShaderSystem)
find_package(OgreProcedural)

# Copy resource.cfg to the build folder. Also if modified will force a reconfiguration:
# https://stackoverflow.com/questions/34799916/copy-file-from-source-directory-to-binary-directory-using-cmake
configure_file( ${CMAKE_SOURCE_DIR}/resources.cfg ${CMAKE_BINARY_DIR}/resources.cfg COPYONLY )

add_executable(cmake_project main.cpp axes_helper_factory.cpp input_manager.cpp end_effector_view.cpp forward_kinematics.cpp inverse_kinematics.cpp collision_selection_manager.cpp object_selection_controller.cpp joint_controller.cpp robot_model.cpp cartesian_controller.cpp robot_view.cpp collision_view.cpp collision_model.cpp point_cloud_view.cpp collision_controller.cpp box_collider.cpp graph_view.cpp path_view.cpp util.cpp motion_planner.cpp naive_nearest_neighbour.cpp trajectory_controller.cpp visibility_controller.cpp motion_planning_controller.cpp test_fcl_utility.cpp)
target_link_libraries(cmake_project ${OGRE_LIBRARIES} ${OIS_LIBRARIES} OgreBites OgreRTShaderSystem ${orocos_kdl_LIBRARIES} fcl ${fcl_LIBRARIES} ${CCD_LIBRARIES} ${OgreProcedural_LIBRARIES})
