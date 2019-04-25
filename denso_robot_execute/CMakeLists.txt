cmake_minimum_required(VERSION 2.8.3)
project(denso_robot_execute)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  control_msgs
  roscpp
  roslaunch
  sensor_msgs
  trajectory_msgs
  )

catkin_package(
  CATKIN_DEPENDS actionlib_msgs
  )

# Build
add_definitions("-std=c++11")

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(execute_action_server src/execute_action_server.cpp)

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
