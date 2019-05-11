cmake_minimum_required(VERSION 2.8.3)
project(denso_robot_execute)

find_package(catkin REQUIRED COMPONENTS
  genmsg
  moveit_commander
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  pluginlib
  actionlib
  actionlib_msgs
  moveit_msgs
  roscpp
  roslaunch
  )

add_message_files(DIRECTORY msg FILES Plan.msg)
add_action_files(DIRECTORY action FILES ExecutePlan.action)
generate_messages(DEPENDENCIES actionlib_msgs moveit_msgs)

catkin_package(
  CATKIN_DEPENDS
  roscpp
  moveit_core
  moveit_ros_planning_interface
  message_runtime
  actionlib_msgs
  moveit_msgs
  )

# Build
add_definitions("-std=c++11")

include_directories(${catkin_INCLUDE_DIRS})

add_executable(execute_action_server src/execute_action_server.cpp)

add_dependencies(execute_action_server execute_action_server_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

target_link_libraries(execute_action_server ${catkin_LIBRARIES})

# Install
foreach(modules execute_action_server)
  install(TARGETS ${modules}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
endforeach(modules)

foreach(dir config launch)
  install(DIRECTORY ${dir}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
