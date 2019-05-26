cmake_minimum_required(VERSION 2.8.3)
project(autobotx)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  message_runtime
  roscpp
  rospy
  std_msgs
  genmsg
tf
nav_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

# find_package(Boost REQUIRED COMPONENTS system)


## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
#catkin_python_setup()



add_message_files(
 FILES
 Unicycle.msg
)

generate_messages(
 DEPENDENCIES
 geometry_msgs  std_msgs
 )


 include_directories(include
   ${catkin_INCLUDE_DIRS}    #this will include ros.h directory and other .h files
   src
 )

 add_library(my_libs
   src/mavlink_communication.cpp
   src/serial_port.cpp
 )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES my_libs
  CATKIN_DEPENDS message_runtime geometry_msgs roscpp rospy std_msgs
)








install(TARGETS my_libs
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

 install(DIRECTORY include
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
 )


 add_executable(link src/link.cpp)
 #target_link_libraries(listner src/mavlink_communication.cpp )
 target_link_libraries(link ${catkin_LIBRARIES}  my_libs)
 add_dependencies(link autobot_generate_messages_cpp)

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
 install(PROGRAMS
 #src/teleop.py
 scripts/teleop_keys.py
 scripts/reader.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )


# Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES
#src/teleop.py
scripts/teleop_keys.py
scripts/reader.py

   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
 )
