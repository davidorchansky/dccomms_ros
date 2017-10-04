cmake_minimum_required(VERSION 2.8.3)
project(dccomms_ros CXX)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
		roscpp
		rospy
		std_msgs
		actionlib
		actionlib_msgs
		roslaunch
		message_generation
                std_srvs
                tf
		dccomms_ros_msgs
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
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
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

# Generate services in the 'srv' folder
# add_service_files(
#   DIRECTORY srv
#   FILES
#   AddDevice.srv
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
#   std_msgs
#   actionlib_msgs
# )
#
################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
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
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES dccomms_ros
  CATKIN_DEPENDS 
	roscpp 
	rospy
	std_msgs 
	actionlib_msgs
        message_runtime
#  DEPENDS system_lib
)

roslaunch_add_file_check(launch)

#http://stackoverflow.com/questions/7787823/cmake-how-to-get-the-name-of-all-subdirectories-of-a-directory
function(GETPROJECTS result curdir)
  FILE(GLOB children RELATIVE ${curdir} ${curdir}/*/CMakeLists.txt)
  SET(dirlist "")
  FOREACH(file ${children})
        get_filename_component(subdir ${file} DIRECTORY)
        LIST(APPEND dirlist ${subdir})
  ENDFOREACH()
  SET(${result} ${dirlist} PARENT_SCOPE)
endfunction()


###########
## Build ##
###########

####### HANDLE DEPENDENCIES ######

SET(${PROJECT_NAME}_DEPS_DIR "modules")
SET(${PROJECT_NAME}_TMP0 ${CMAKE_CURRENT_SOURCE_DIR}/${${PROJECT_NAME}_DEPS_DIR})
GETPROJECTS(${PROJECT_NAME}_DEPS ${${PROJECT_NAME}_TMP0})
message(STATUS "Dependencies found in ${${PROJECT_NAME}_TMP0}: ${${PROJECT_NAME}_DEPS}")
FOREACH(subdir ${${PROJECT_NAME}_DEPS})
        add_subdirectory(${${PROJECT_NAME}_DEPS_DIR}/${subdir})
ENDFOREACH()

SET(${PROJECT_NAME}_CXX_COMPILE_FLAGS "-fPIC -std=c++14 ")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${${PROJECT_NAME}_CXX_COMPILE_FLAGS}")

SET(${PROJECT_NAME}_INCLUDE_DIRS
    "include"
    "modules/ns-3-dev/build")

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


SET(S100BRIDGE s100_bridge)
add_executable(${S100BRIDGE} src/bridges/s100/s100_main.cpp)
target_include_directories(${S100BRIDGE} PUBLIC ${${PROJECT_NAME}_INCLUDE_DIRS})
target_link_libraries(${S100BRIDGE} ${DCCOMMS_UTILS_GIT} ${catkin_LIBRARIES})

###############################################
####### UNCOMMENT THE FOLLOWING BLOCK #########
#######     TO ENABLE netsim BUILD    #########
###############################################

#SET(${PROJECT_NAME}_CPP_FILES
#        src/simulator/ROSCommsDevice.cpp
#        src/simulator/ROSCommsSimulator.cpp
#        src/simulator/ROSCommsChannelState.cpp
#)
#
#add_library(${PROJECT_NAME} SHARED ${${PROJECT_NAME}_CPP_FILES})
#target_include_directories(${PROJECT_NAME} PUBLIC ${${PROJECT_NAME}_INCLUDE_DIRS})
#target_link_libraries(${PROJECT_NAME} ${DCCOMMS_UTILS_GIT} ${catkin_LIBRARIES})
#
#SET(NS3_BUILD "modules/ns-3-dev/build")
#find_library(NS3_CORE_LIB libns3-dev-core-debug.so HINTS ${NS3_BUILD})
#find_library(NS3_NETWORK_LIB libns3-dev-network-debug.so HINTS ${NS3_BUILD})
#SET(NS3_LIBS
#    ${NS3_CORE_LIB}
#    ${NS3_NETWORK_LIB}
#    )
#
#SET(SIM netsim)
#add_executable (${SIM} src/simulator/main/netsim.cpp)
#target_include_directories(${SIM} PUBLIC ${${PROJECT_NAME}_INCLUDE_DIRS})
#target_link_libraries(${SIM} ${PROJECT_NAME} ${DCCOMMS_UTILS_GIT} ${catkin_LIBRARIES}
#     ${NS3_LIBS})
#
#add_executable (ns3-testing testing/simulator/ns3-testing.cpp)
##target_compile_options(pruebas PUBLIC "")
##https://cmake.org/cmake/help/v3.4/command/find_library.html
##https://stackoverflow.com/questions/31438916/cmake-cannot-find-library-using-link-directories
#
#target_link_libraries(ns3-testing ${NS3_LIBS} ${PROJECT_NAME})
#target_include_directories(ns3-testing PUBLIC
#    modules/ns-3-dev/build
#    ${CMAKE_CURRENT_SOURCE_DIR}
#    )

###############################################
############ END netsim BUILD #################
###############################################


###############################################
####### UNCOMMENT THE FOLLOWING BLOCK #########
#######  TO ENABLE comms tests BUILD  #########
###############################################
add_executable (test_op testing/comms/op.cpp)
target_link_libraries(test_op
    ${DCCOMMS_UTILS_GIT}
    ${catkin_LIBRARIES}
    )
target_include_directories(test_op PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    )

add_executable (test_rov testing/comms/rov.cpp)
target_link_libraries(test_rov
    ${DCCOMMS_UTILS_GIT}
    ${catkin_LIBRARIES}
    )
target_include_directories(test_rov PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    )
###############################################
########### END comms tests BUILD #############
###############################################

message("End ${PROJECT_NAME_ORIG}")
