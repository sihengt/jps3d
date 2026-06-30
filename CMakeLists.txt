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
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(octomap REQUIRED)

PKG_CHECK_MODULES(YAMLCPP REQUIRED yaml-cpp)
if(YAMLCPP_FOUND)
  message(STATUS "Found yaml-cpp.")
else()
  message(FATAL_ERROR "Could not find yaml-cpp.")
endif()

include_directories(include ${EIGEN3_INCLUDE_DIRS})

add_library(jps_lib src/jps_planner/graph_search.cpp
  src/jps_planner/jps_planner.cpp)
target_compile_options(jps_lib PRIVATE -Wno-implicit-fallthrough)

add_library(dmp_lib src/distance_map_planner/graph_search.cpp
  src/distance_map_planner/distance_map_planner.cpp)

add_executable(create_map test/create_map.cpp)
target_link_libraries(create_map ${YAMLCPP_LIBRARIES})

add_executable(jps3d_node src/jps3d_node.cpp)
target_include_directories(jps3d_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(jps3d_node PUBLIC c_std_99 cxx_std_17)
target_include_directories(jps3d_node PUBLIC ${OCTOMAP_INCLUDE_DIRS})
target_link_libraries(jps3d_node jps_lib ${OCTOMAP_LIBRARIES})
ament_target_dependencies(jps3d_node
  rclcpp nav_msgs geometry_msgs sensor_msgs visualization_msgs std_msgs
  octomap_msgs octomap)

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

add_executable(test_planner_ceiling test/test_planner_ceiling.cpp)
target_link_libraries(test_planner_ceiling jps_lib ${YAMLCPP_LIBRARIES})
add_test(test_planner_ceiling test_planner_ceiling ${CMAKE_SOURCE_DIR}/data/simple3d.yaml)

# add_executable(test_distance_map_planner_2d test/test_distance_map_planner_2d.cpp)
# target_link_libraries(test_distance_map_planner_2d jps_lib dmp_lib ${YAMLCPP_LIBRARIES} ${Boost_LIBRARIES})
# add_test(test_distance_map_planner_2d test_distance_map_planner_2d ${CMAKE_SOURCE_DIR}/data/corridor.yaml)


install(FILES "${PROJECT_NAME}Config.cmake" "${PROJECT_NAME}ConfigVersion.cmake"
  DESTINATION "share/${PROJECT_NAME}/cmake")

install(TARGETS jps_lib dmp_lib
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

install(DIRECTORY include/jps_basis include/jps_planner include/jps_collision
  DESTINATION include)

install(TARGETS jps3d_node
  DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS
  scripts/rviz_goal_client.py
  scripts/waypoint_sequencer.py
  scripts/waypoint_sequencer_logic.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()