cmake_minimum_required(VERSION 3.0.2)

#set the project name
project(final_pkg)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  robotics_hw1
  message_filters
  dynamic_reconfigure
  message_generation
        tf
        tf2
        tf2_ros
)

generate_dynamic_reconfigure_options(
  cfg/parameters.cfg
)

add_message_files(
 FILES
 OdometryWithIntegrationMethod.msg
)

add_service_files(
  FILES
  ResetOdometryToZero.srv
  ResetOdometryToGivenPose.srv
)

 
generate_messages(
   DEPENDENCIES
  std_msgs nav_msgs
)

# Declare that this catkin package's runtime dependencies
catkin_package(
 CATKIN_DEPENDS geometry_msgs nav_msgs std_msgs robotics_hw1 message_filters message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(pub_velocities src/pub_velocities.cpp)
target_link_libraries(pub_velocities ${catkin_LIBRARIES})

add_executable(pub_odometry src/pub_odometry.cpp)
add_dependencies(pub_odometry ${PROJECT_NAME}_gencfg)
add_dependencies(pub_odometry robotics_project1_generate_messages_cpp)
target_link_libraries(pub_odometry ${catkin_LIBRARIES})
add_dependencies(pub_odometry ${catkin_EXPORTED_TARGETS})
