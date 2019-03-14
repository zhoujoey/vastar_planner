cmake_minimum_required(VERSION 2.8.3)
project(waypoint_planner)
add_compile_options(-std=c++11)
find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    tf
    nav_core
    costmap_2d
    dynamic_reconfigure
    pluginlib
    rospy
    std_msgs
    shield_msgs
  )

generate_dynamic_reconfigure_options(
  cfg/WaypointPlanner.cfg
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  )
add_definitions(${EIGEN3_DEFINITIONS})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES waypoint_planner
  CATKIN_DEPENDS
    roscpp
    pluginlib
    costmap_2d
    dynamic_reconfigure
    nav_core
    rospy
    std_msgs
    shield_msgs
)
#set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")
add_library(waypoint_planner src/waypoint_planner.cpp src/vastar.cpp)
add_dependencies(waypoint_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(waypoint_planner
    ${catkin_LIBRARIES}
    )

install(TARGETS waypoint_planner
       ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       )

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

install(FILES bgp_plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


