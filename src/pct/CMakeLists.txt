cmake_minimum_required(VERSION 2.8.3)
project(pct)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17 -O3)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  roslib
  rospy
  std_msgs
  gen_utilities
  robot_utilities
)

## System dependencies are found with CMake's conventions

# find_package(CGAL REQUIRED)
# if ( CGAL_FOUND )
#   message(STATUS "CGAL_FOUND")
# else()
#     message(STATUS "This program requires the CGAL library, and will not be compiled.")
# endif()
# find_package(GMP REQUIRED)


find_package(orocos_kdl REQUIRED)
find_package(Boost 1.54.0 REQUIRED COMPONENTS system filesystem)

# find_package(gen_utilities REQUIRED)
# find_package(robot_utilities REQUIRED)

# find_package(libnlopt REQUIRED COMPONENTS)
find_package(fcl REQUIRED)
find_package(CCD REQUIRED)

# find_package(libnabo REQUIRED COMPONENTS)
# find_package(libigl REQUIRED COMPONENTS)

set(libigl_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../libigl)
set(libnlopt_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../libnlopt)
set(libnabo_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../libnabo)

# set(fcl_INCLUDE_DIRS /usr/local/include)
# set(FCL_LIB /usr/local/lib/libfcl.so)
# message(WARNING "FCL INCLUDES LOCATED AT ${fcl_INCLUDE_DIRS}")
set(fcl_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/../fcl/install/include)
set(FCL_LIB ${CMAKE_CURRENT_SOURCE_DIR}/../fcl/install/lib/libfcl.so)
message(WARNING "FCL INCLUDES LOCATED AT ${fcl_INCLUDE_DIRS}")
# set(FCL_INCLUDE_DIRS
# ${CMAKE_CURRENT_SOURCE_DIR}/../fcl/build/install/include)
# set(FCL_LIB ${CMAKE_CURRENT_SOURCE_DIR}/../fcl/build/install/lib/libfcl.a)
# message(WARNING "FCL INCLUDES LOCATED AT ${FCL_INCLUDE_DIRS}")

# Adding nlopt header file and linking dependancies 
include_directories(${libnlopt_SOURCE_DIR}/install/include)
link_directories(${libnlopt_SOURCE_DIR}/install/lib)
set(nlopt_ADDED_LIBS ${libnlopt_SOURCE_DIR}/install/lib/libnlopt.so)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

find_package(Eigen3)
if(NOT EIGEN3_FOUND)
  # Fallback to cmake_modules
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
  set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})  # Not strictly necessary as Eigen is head only
  # Possibly map additional variables to the EIGEN3_ prefix.
else()
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

if (fcl_FOUND)
  message("fcl found!")
  message("${fcl_INCLUDE_DIRS}")
    include_directories(${fcl_INCLUDE_DIRS})
endif (fcl_FOUND) 

if (CCD_FOUND)
  message("CCD found!")
  message("${CCD_INCLUDE_DIRS}")
  include_directories(${CCD_INCLUDE_DIRS})
endif (CCD_FOUND)


# ## This setting causes Qt's "MOC" generation to happen automatically.
# set(CMAKE_AUTOMOC ON)

# ## This plugin includes Qt widgets, so we must include Qt.
# ## We'll use the version that rviz used so they are compatible.
# if(rviz_QT_VERSION VERSION_LESS "5")
#   message(STATUS "Using Qt4 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
#   find_package(Qt4 ${rviz_QT_VERSION} EXACT REQUIRED QtCore QtGui)
#   ## pull in all required include dirs, define QT_LIBRARIES, etc.
#   include(${QT_USE_FILE})
# else()
#   message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
#   find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
#   ## make target_link_libraries(${QT_LIBRARIES}) pull in all required dependencies
#   set(QT_LIBRARIES Qt5::Widgets)
# endif()

# ## I prefer the Qt signals and slots to avoid defining "emit", "slots",
# ## etc because they can conflict with boost signals, so define QT_NO_KEYWORDS here.
# add_definitions(-DQT_NO_KEYWORDS)

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

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   geometry_msgs#   std_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES pct
#  CATKIN_DEPENDS geometry_msgs roscpp roslib rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${libnlopt_SOURCE_DIR}/install/include
  ${libnabo_SOURCE_DIR}/include/libnabo
  ${libigl_SOURCE_DIR}/include/libigl
  ${orocos_kdl_INCLUDE_DIRS}
  /opt/ros/kinetic/include/kdl
  /opt/ros/kinetic/include/urdf
  /opt/ros/kinetic/include
  ${fcl_INCLUDE_DIRS}
#   CGAL_CreateSingleSourceCGALProgram
#   /opt/gurobi901/linux64/
#   /opt/gurobi901/linux64/include
#   /opt/gurobi901/linux64/bin
)

link_directories(${libnabo_SOURCE_DIR}/include/libnabo/build/libnabo.a
                ${libigl_SOURCE_DIR}/include/libigl/libigl.a
                /opt/ros/kinetic/lib
                /usr/local/lib
#                 /opt/gurobi901/linux64/lib/
                )


set(DEP_LIBS ${catkin_LIBRARIES}
              ${FCL_LIB} ${EIGEN3_LIBRARIES}
              /usr/lib/x86_64-linux-gnu/libboost_iostreams.a
              /usr/lib/x86_64-linux-gnu/libboost_system.a
              /usr/lib/x86_64-linux-gnu/libboost_filesystem.a
              /usr/lib/x86_64-linux-gnu/libutil.a
              /usr/lib/x86_64-linux-gnu/libboost_thread.a
              ${libnabo_SOURCE_DIR}/include/libnabo/build/libnabo.a gomp
              orocos-kdl kdl_parser urdf
              ${Boost_LIBRARIES}
              ${nlopt_ADDED_LIBS}
#               ${WITH_CGAL_Core} ${WITH_CGAL_ImageIO} ${WITH_CGAL_Qt5}
#               gmp
              ${Boost_LIBRARIES}
#               /opt/gurobi901/linux64/lib/libgurobi_g++5.2.a
#               /opt/gurobi901/linux64/lib/libgurobi90.so
                      )

add_executable(main src/main.cpp)
target_link_libraries(main ${DEP_LIBS})

add_executable(analysis src/analysis.cpp)
target_link_libraries(analysis ${DEP_LIBS})

add_executable(test_coll src/test_coll.cpp)
target_link_libraries(test_coll ${DEP_LIBS})

add_executable(path_T_traj src/path_T_traj.cpp)
target_link_libraries(path_T_traj ${DEP_LIBS})


# add_executable(test_qp src/test_qp.cpp src/irb2600.cpp)
# target_link_libraries(test_qp ${DEP_LIBS})

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/pct.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/pct_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_pct.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
