cmake_minimum_required(VERSION 2.8.3)
project(dccomms_ros CXX)

set(CMAKE_VERBOSE_MAKEFILE ON)

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

####### HANDLE DEPENDENCIES ######

SET(${PROJECT_NAME}_DEPS_DIR "modules")
SET(${PROJECT_NAME}_TMP0 ${CMAKE_CURRENT_SOURCE_DIR}/${${PROJECT_NAME}_DEPS_DIR})
GETPROJECTS(${PROJECT_NAME}_DEPS ${${PROJECT_NAME}_TMP0})
message(STATUS "Dependencies found in ${${PROJECT_NAME}_TMP0}: ${${PROJECT_NAME}_DEPS}")
FOREACH(subdir ${${PROJECT_NAME}_DEPS})
        add_subdirectory(${${PROJECT_NAME}_DEPS_DIR}/${subdir})
ENDFOREACH()

SET(NS3_BUILD ${CMAKE_CURRENT_SOURCE_DIR}/modules/ns-3-dev/build/)

SET(NS3_LIBS
    libns3-dev-antenna-debug.so                libns3-dev-flow-monitor-test-debug.so      libns3-dev-propagation-debug.so
    libns3-dev-antenna-test-debug.so           libns3-dev-internet-apps-debug.so          libns3-dev-propagation-test-debug.so
    libns3-dev-aodv-debug.so                   libns3-dev-internet-apps-test-debug.so     libns3-dev-sixlowpan-debug.so
    libns3-dev-aodv-test-debug.so              libns3-dev-internet-debug.so               libns3-dev-sixlowpan-test-debug.so
    libns3-dev-applications-debug.so           libns3-dev-internet-test-debug.so          libns3-dev-spectrum-debug.so
    libns3-dev-applications-test-debug.so      libns3-dev-lr-wpan-debug.so                libns3-dev-spectrum-test-debug.so
    libns3-dev-aqua-sim-ng-debug.so            libns3-dev-lr-wpan-test-debug.so           libns3-dev-stats-debug.so
    libns3-dev-aqua-sim-ng-test-debug.so       libns3-dev-lte-debug.so                    libns3-dev-stats-test-debug.so
    libns3-dev-bridge-debug.so                 libns3-dev-lte-test-debug.so               libns3-dev-tap-bridge-debug.so
    libns3-dev-buildings-debug.so              libns3-dev-mesh-debug.so                   libns3-dev-test-debug.so
    libns3-dev-buildings-test-debug.so         libns3-dev-mesh-test-debug.so              libns3-dev-test-test-debug.so
    libns3-dev-config-store-debug.so           libns3-dev-mobility-debug.so               libns3-dev-topology-read-debug.so
    libns3-dev-core-debug.so                   libns3-dev-mobility-test-debug.so          libns3-dev-topology-read-test-debug.so
    libns3-dev-core-test-debug.so              libns3-dev-mpi-debug.so                    libns3-dev-traffic-control-debug.so
    libns3-dev-csma-debug.so                   libns3-dev-netanim-debug.so                libns3-dev-traffic-control-test-debug.so
    libns3-dev-csma-layout-debug.so            libns3-dev-netanim-test-debug.so           libns3-dev-uan-debug.so
    libns3-dev-dsdv-debug.so                   libns3-dev-network-debug.so                libns3-dev-uan-test-debug.so
    libns3-dev-dsdv-test-debug.so              libns3-dev-network-test-debug.so           libns3-dev-virtual-net-device-debug.so
    libns3-dev-dsr-debug.so                    libns3-dev-nix-vector-routing-debug.so     libns3-dev-wave-debug.so
    libns3-dev-dsr-test-debug.so               libns3-dev-olsr-debug.so                   libns3-dev-wave-test-debug.so
    libns3-dev-energy-debug.so                 libns3-dev-olsr-test-debug.so              libns3-dev-wifi-debug.so
    libns3-dev-energy-test-debug.so            libns3-dev-point-to-point-debug.so         libns3-dev-wifi-test-debug.so
    libns3-dev-fd-net-device-debug.so          libns3-dev-point-to-point-layout-debug.so  libns3-dev-wimax-debug.so
    libns3-dev-flow-monitor-debug.so           libns3-dev-point-to-point-test-debug.so    libns3-dev-wimax-test-debug.so
)

#Catkin do not export well relative paths to libraries: https://github.com/ros/catkin/issues/535
#link_directories(${NS3_BUILD})
#So it's better to specify the full path to each .so if the the *.so are not in the common directories (see ldconfig)
message("NS3 modules: ${NS3_LIBS}")
SET(ns3liblist "")
FOREACH(lib ${NS3_LIBS})
SET(path ${NS3_BUILD}/${lib})
LIST(APPEND ns3liblist ${path})
ENDFOREACH()

SET(${PROJECT_NAME}_INCLUDE_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}/src/include
    ${CMAKE_CURRENT_SOURCE_DIR}/modules/ns-3-dev/build
    ${${DCCOMMS_UTILS_GIT}_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    )

SET(${PROJECT_NAME}_CXX_COMPILE_FLAGS "-fPIC -std=c++14 ")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${${PROJECT_NAME}_CXX_COMPILE_FLAGS}")

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
  INCLUDE_DIRS ${${PROJECT_NAME}_INCLUDE_DIRS}
  LIBRARIES ${PROJECT_NAME} ${${DCCOMMS_UTILS_GIT}_LIBRARIES} ${ns3liblist}
  CATKIN_DEPENDS
        roscpp
        rospy
        std_msgs
        actionlib_msgs
        message_runtime
        dccomms_ros_msgs
#  DEPENDS system_lib
)

roslaunch_add_file_check(launch)

SET(S100BRIDGE s100_bridge)
add_executable(${S100BRIDGE} src/bridges/s100/s100_main.cpp)
target_include_directories(${S100BRIDGE} PUBLIC ${${PROJECT_NAME}_INCLUDE_DIRS})
target_link_libraries(${S100BRIDGE} ${DCCOMMS_UTILS_GIT} ${catkin_LIBRARIES})

SET(${PROJECT_NAME}_CPP_FILES
        src/simulator/ROSCommsDevice.cpp
        src/simulator/ROSCommsSimulator.cpp
        src/simulator/AcousticROSCommsDevice.cpp
        src/simulator/CustomROSCommsDevice.cpp
        src/simulator/AcousticCommsChannel.cpp
        src/simulator/CustomCommsChannel.cpp
)

add_library(${PROJECT_NAME} SHARED ${${PROJECT_NAME}_CPP_FILES})
target_include_directories(${PROJECT_NAME} PUBLIC ${${PROJECT_NAME}_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}  ${ns3liblist} ${DCCOMMS_UTILS_GIT} ${catkin_LIBRARIES})


add_executable (netsim_example1 src/simulator/main/netsim_example1.cpp)
target_link_libraries(netsim_example1 ${PROJECT_NAME})

add_executable (netsim_example2 src/simulator/main/netsim_example2.cpp)
target_link_libraries(netsim_example2 ${PROJECT_NAME})

add_executable (netsim_example3 src/simulator/main/netsim_example3.cpp)
target_link_libraries(netsim_example3 ${PROJECT_NAME})

#target_compile_options(pruebas PUBLIC "")
#https://cmake.org/cmake/help/v3.4/command/find_library.html
#https://stackoverflow.com/questions/31438916/cmake-cannot-find-library-using-link-directories
add_executable (ns3-testing testing/simulator/ns3-testing.cpp)
target_link_libraries(ns3-testing ${PROJECT_NAME})
target_include_directories(ns3-testing PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

add_executable (ns3-as-testing testing/simulator/ns3-as-testing.cpp)
target_link_libraries(ns3-as-testing ${PROJECT_NAME})
target_include_directories(ns3-as-testing PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

add_executable (ns3-as-VBF testing/simulator/ns3-as-VBF.cpp)
target_link_libraries(ns3-as-VBF ${PROJECT_NAME})
target_include_directories(ns3-as-VBF PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
###############################################
############ END netsim BUILD #################
###############################################


message("End ${PROJECT_NAME_ORIG}")
