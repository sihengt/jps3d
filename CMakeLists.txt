cmake_minimum_required(VERSION 3.8)
project(jps3d)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -Wall -Wno-deprecated-declarations")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

set(BUILD_SHARED_LIBS ON)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(octomap REQUIRED)
# ROGMapUtil (include/jps_collision/map_util_super.h) is installed for
# downstream use, so rog_map has to be resolvable for consumers; it is already
# declared in package.xml and re-exported below.
find_package(rog_map REQUIRED)

PKG_CHECK_MODULES(YAMLCPP REQUIRED yaml-cpp)
if(YAMLCPP_FOUND)
  message(STATUS "Found yaml-cpp.")
else()
  message(FATAL_ERROR "Could not find yaml-cpp.")
endif()

include_directories(include ${EIGEN3_INCLUDE_DIRS})

add_library(jps_lib src/jps_planner/graph_search.cpp
  src/jps_planner/jps_planner.cpp
  src/jps3d_frontend.cpp)
target_compile_options(jps_lib PRIVATE -Wno-implicit-fallthrough)
target_include_directories(jps_lib PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# TODO: dmp_lib (distance_map_planner) is currently broken independent of the
# standalone-build work here (pre-existing map_util_/TmapValue-vs-Vecf type
# errors) -- disabled until fixed in a separate pass.
# add_library(dmp_lib src/distance_map_planner/graph_search.cpp
#   src/distance_map_planner/distance_map_planner.cpp)

# TODO: remove ros dependency for now
# add_executable(create_map test/create_map.cpp)
# target_link_libraries(create_map ${YAMLCPP_LIBRARIES})

add_executable(jps3d_node src/jps3d_node.cpp)
target_include_directories(jps3d_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(jps3d_node PUBLIC c_std_99 cxx_std_17)
target_include_directories(jps3d_node PUBLIC ${OCTOMAP_INCLUDE_DIRS})
target_link_libraries(jps3d_node jps_lib ${OCTOMAP_LIBRARIES})
ament_target_dependencies(jps3d_node
  rclcpp nav_msgs geometry_msgs visualization_msgs std_msgs octomap_msgs)

add_executable(octomap_test_publisher test/octomap_test_publisher.cpp)
target_include_directories(octomap_test_publisher PUBLIC ${OCTOMAP_INCLUDE_DIRS})
target_link_libraries(octomap_test_publisher ${OCTOMAP_LIBRARIES})
ament_target_dependencies(octomap_test_publisher rclcpp octomap_msgs)

include(CTest)

add_executable(test_planner_2d test/test_planner_2d.cpp)
target_link_libraries(test_planner_2d jps_lib ${YAMLCPP_LIBRARIES})
add_test(test_planner_2d test_planner_2d ${CMAKE_SOURCE_DIR}/data/corridor.yaml)

add_executable(test_planner_3d test/test_planner_3d.cpp)
target_link_libraries(test_planner_3d jps_lib ${YAMLCPP_LIBRARIES})
add_test(test_planner_3d test_planner_3d ${CMAKE_SOURCE_DIR}/data/simple3d.yaml)

add_executable(test_planner_replan test/test_planner_replan.cpp)
target_link_libraries(test_planner_replan jps_lib ${YAMLCPP_LIBRARIES})
add_test(test_planner_replan test_planner_replan ${CMAKE_SOURCE_DIR}/data/simple3d.yaml)

add_executable(test_map_util_ceiling test/test_map_util_ceiling.cpp)
target_link_libraries(test_map_util_ceiling jps_lib)
add_test(test_map_util_ceiling test_map_util_ceiling)

add_executable(test_map_util_dilate test/test_map_util_dilate.cpp)
target_link_libraries(test_map_util_dilate jps_lib)
add_test(test_map_util_dilate test_map_util_dilate)

add_executable(test_map_util_octo test/test_map_util_octo.cpp)
target_include_directories(test_map_util_octo PRIVATE ${OCTOMAP_INCLUDE_DIRS})
target_link_libraries(test_map_util_octo jps_lib ${OCTOMAP_LIBRARIES})
add_test(test_map_util_octo test_map_util_octo)

add_executable(test_jps3d_frontend test/test_jps3d_frontend.cpp)
target_link_libraries(test_jps3d_frontend jps_lib)
add_test(test_jps3d_frontend test_jps3d_frontend)

add_executable(test_planner_ceiling test/test_planner_ceiling.cpp)
target_link_libraries(test_planner_ceiling jps_lib ${YAMLCPP_LIBRARIES})
add_test(test_planner_ceiling test_planner_ceiling ${CMAKE_SOURCE_DIR}/data/simple3d.yaml)

# add_executable(test_distance_map_planner_2d test/test_distance_map_planner_2d.cpp)
# target_link_libraries(test_distance_map_planner_2d jps_lib dmp_lib ${YAMLCPP_LIBRARIES} ${Boost_LIBRARIES})
# add_test(test_distance_map_planner_2d test_distance_map_planner_2d ${CMAKE_SOURCE_DIR}/data/corridor.yaml)


# TODO: ros stuff
# install(FILES "${PROJECT_NAME}Config.cmake" "${PROJECT_NAME}ConfigVersion.cmake"
#   DESTINATION "share/${PROJECT_NAME}/cmake")

install(TARGETS jps_lib
  EXPORT export_jps3d
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

install(DIRECTORY include/jps_basis include/jps_planner include/jps_collision
  include/jps3d
  DESTINATION include)

ament_export_include_directories(include)
ament_export_targets(export_jps3d)
ament_export_dependencies(rog_map)

install(TARGETS jps3d_node
  DESTINATION lib/${PROJECT_NAME})

# install(PROGRAMS
#   scripts/rviz_goal_client.py
#   scripts/waypoint_sequencer.py
#   scripts/waypoint_sequencer_logic.py
#   DESTINATION lib/${PROJECT_NAME}
# )

# install(DIRECTORY launch/
#   DESTINATION share/${PROJECT_NAME}/launch
# )
# install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

# ---------------------------------------------------------------------------
# Optional ROG-Map backend (ROGMapUtil in include/jps_collision/map_util_super.h)
# Requires a built rog_map package on CMAKE_PREFIX_PATH, e.g.
#   -DJPS3D_WITH_ROG_MAP=ON -DCMAKE_PREFIX_PATH=<ws>/install_local/rog_map
option(JPS3D_WITH_ROG_MAP "Build ROGMap-backed tests and benchmark" OFF)
if(JPS3D_WITH_ROG_MAP)
  # search counters (GraphSearch::Stats) for the benchmark
  target_compile_definitions(jps_lib PUBLIC JPS_PROFILE)
  find_package(PCL REQUIRED COMPONENTS common io)
  find_package(pcl_conversions REQUIRED)
  find_package(tf2_ros REQUIRED)
  find_package(tf2_sensor_msgs REQUIRED)

  function(jps3d_rog_target name)
    add_executable(${name} ${ARGN})
    target_include_directories(${name} PRIVATE
      ${rog_map_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
    # rog_map was compiled with ORIGIN_AT_CORNER; its inline index math in
    # headers must see the same definition.
    target_compile_definitions(${name} PRIVATE ORIGIN_AT_CORNER JPS_PROFILE
      ROOT_DIR="${rog_map_DIR}/../../../../../rog_map/")
    target_link_libraries(${name} jps_lib ${rog_map_LIBRARIES}
      ${PCL_LIBRARIES} ${YAMLCPP_LIBRARIES})
    ament_target_dependencies(${name}
      rclcpp std_msgs nav_msgs geometry_msgs visualization_msgs
      pcl_conversions tf2_ros tf2_sensor_msgs)
  endfunction()

  jps3d_rog_target(test_map_util_super test/test_map_util_super.cpp
    src/jps3d_frontend.cpp)
  add_test(test_map_util_super test_map_util_super
    ${CMAKE_SOURCE_DIR}/test/data/rog_map_bench.yaml)

  jps3d_rog_target(test_planner_3d_rog_map test/test_planner_3d_rog_map.cpp)
  add_test(test_planner_3d_rog_map test_planner_3d_rog_map
    ${CMAKE_SOURCE_DIR}/test/data/rog_map_bench_pcd.yaml)

  jps3d_rog_target(bench_jps_rog_map test/bench_jps_rog_map.cpp
    src/jps3d_frontend.cpp)
endif()

ament_package()