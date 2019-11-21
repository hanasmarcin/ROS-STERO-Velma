cmake_minimum_required(VERSION 2.8)
project(stero_velma)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  geometry_msgs
  nav_msgs
  message_generation
  tf2_ros
  tf
  costmap_2d
  global_planner
  dwa_local_planner
  nav_core
  std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)


add_service_files(
  FILES
  lab1_1.srv
)


generate_messages(
  DEPENDENCIES
  nav_msgs
)

catkin_package()

install(DIRECTORY data/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/data
  )

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  )

catkin_install_python(PROGRAMS
    scripts/hello_world
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
