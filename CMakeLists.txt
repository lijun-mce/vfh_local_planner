cmake_minimum_required(VERSION 3.5)
project(vfh_local_planner)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
add_compile_options(-g)

# Set to Release in order to speed up the program significantly
set(CMAKE_BUILD_TYPE Release) #None, Debug, Release, RelWithDebInfo, MinSizeRel
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

## Find catkin macros and libraries
find_package(ament_cmake REQUIRED)
find_package(nav_2d_utils REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pluginlib REQUIRED)

###########
## Build ##
###########

include_directories(
  include
)

## Declare a C++ library
add_library(${PROJECT_NAME} SHARED
  src/vfh_local_planner_ros.cpp
  src/vfh_local_planner.cpp
  src/utils.cpp
  src/vfh_config.cpp
)

set(ament_dependencies
    ament_cmake
    nav_2d_utils
    nav2_core
    nav2_costmap_2d
    nav2_util
    geometry_msgs
    nav_msgs
    rclcpp
    rclcpp_action
    rclcpp_lifecycle
    std_msgs
    tf2
    tf2_eigen
    tf2_geometry_msgs
    tf2_ros
    pluginlib
)

ament_target_dependencies(vfh_local_planner
   ${ament_dependencies}
)
# target_link_libraries(teb_local_planner
#    ${EXTERNAL_LIBS}
# )

target_compile_definitions(vfh_local_planner PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")


#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS vfh_local_planner
   DESTINATION lib
)

## Mark cpp header files for installation
install(DIRECTORY include/
   DESTINATION include/
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES
  vfh_local_planner_plugin.xml
  DESTINATION share
  )

install(DIRECTORY params DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(vfh_local_planner)
ament_export_dependencies(${ament_dependencies})
pluginlib_export_plugin_description_file(nav2_core vfh_local_planner_plugin.xml)


#############
## Testing ##
#############

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  #ament_add_gtest(homotopy_class_planner_test
  #    test/homotopy_class_planner_test.cpp
  #)
  #target_link_libraries(homotopy_class_planner_test
  #    teb_local_planner
  #)
endif()

ament_package()