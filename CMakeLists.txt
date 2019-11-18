cmake_minimum_required(VERSION 2.8)
project(stero_velma)

find_package(catkin REQUIRED)

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
