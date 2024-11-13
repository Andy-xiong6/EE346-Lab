cmake_minimum_required(VERSION 3.0.2)
project(Lab)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  geometry_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(homing_task_node src/homing_task.cpp)

target_link_libraries(homing_task_node ${catkin_LIBRARIES})

add_dependencies(homing_task_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


