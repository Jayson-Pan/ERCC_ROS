cmake_minimum_required(VERSION 3.0.2)
project(rescue_task)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  apriltag_ros
  tf
  actionlib
  move_base_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs sensor_msgs apriltag_ros tf actionlib move_base_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Add C++ executables
add_executable(fine_tuning_test src/fine_tuning_test.cpp)
target_link_libraries(fine_tuning_test ${catkin_LIBRARIES})
add_dependencies(fine_tuning_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(rescue_controller src/rescue_controller.cpp)
target_link_libraries(rescue_controller ${catkin_LIBRARIES})
add_dependencies(rescue_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# Install C++ executables
install(TARGETS fine_tuning_test rescue_controller
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

