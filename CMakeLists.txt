cmake_minimum_required(VERSION 2.8.3)
project(dccomms_ros CXX)

#It seams that from Ubuntu 17.10 the linker flag --disable-new-dtags is needed
#for the executable to be able to find absolute linked shared libraries with rpath.
#https://stackoverflow.com/questions/47117443/dynamic-linking-with-rpath-not-working-under-ubuntu-17-10
SET(CMAKE_EXE_LINKER_FLAGS "-Wl,--disable-new-dtags")

set(CMAKE_VERBOSE_MAKEFILE ON)

SET(MODULES_DIR "modules" )

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
		roscpp
		rospy
		std_msgs
		actionlib
		actionlib_msgs
		message_generation
                std_srvs
                tf
                dccomms_ros_msgs
)

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

#BEGIN RESOLVE GIT REVISION
#https://bravenewmethod.com/2017/07/14/git-revision-as-compiler-definition-in-build-with-cmake/
message(STATUS "Resolving GIT Version...")
set(_build_version "unknown")
find_package(Git)
if(GIT_FOUND)
  execute_process(
    COMMAND ${GIT_EXECUTABLE} rev-parse --short HEAD
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
    OUTPUT_VARIABLE _build_version
    ERROR_QUIET
    OUTPUT_STRIP_TRAILING_WHITESPACE
  )
  message( STATUS "GIT hash: ${_build_version}")
else()
  message(STATUS "GIT not found")
endif()
string(TIMESTAMP _time_stamp)
#END RESOLVE GIT REVISION

####### HANDLE DEPENDENCIES ######
SET(DEPS_DIR ${MODULES_DIR})
SET(TMP0 ${CMAKE_CURRENT_SOURCE_DIR}/${DEPS_DIR})
GETPROJECTS(DEPS ${TMP0})
message(STATUS "Dependencies found in ${TMP0}: ${DEPS}")
FOREACH(subdir ${DEPS})
if(GIT_FOUND)
  SET(SUBPROJECT_PATH ${DEPS_DIR}/${subdir})
  SET(SUBPROJECT_FULL_PATH ${CMAKE_CURRENT_SOURCE_DIR}/${SUBPROJECT_PATH})
  execute_process(
    COMMAND ${GIT_EXECUTABLE} rev-parse --short HEAD
    WORKING_DIRECTORY ${SUBPROJECT_FULL_PATH}
    OUTPUT_VARIABLE _sub_build_version
    ERROR_QUIET
    OUTPUT_STRIP_TRAILING_WHITESPACE
  )
  SET(SUBPROJECT_GIT_NAME ${subdir}_${_sub_build_version})
  message(STATUS "Subproject generated git name: " ${SUBPROJECT_GIT_NAME})
  if (TARGET ${subdir})
      message(WARNING "Subproject ${SUBPROJECT_GIT_NAME} (${subdir}) has already been built!")
  else()
      add_subdirectory(${SUBPROJECT_PATH})
  endif()
else()
  message(STATUS "GIT not found")
endif()
ENDFOREACH()


SET(NS3_BUILD ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/ns3)
SET(NS3_INCLUDE_PATH ${NS3_BUILD})
SET(NS3_HEADERS_DIR ${NS3_BUILD}/ns3)

message("NS3 build path: ${NS3_BUILD}")
message("NS3 include path: ${NS3_INCLUDE_PATH}")
message("NS3 headers dir: ${NS3_HEADERS_DIR}")
execute_process(
  COMMAND mkdir -p ${NS3_BUILD}
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)

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
    ${NS3_INCLUDE_PATH}
    ${CMAKE_CURRENT_SOURCE_DIR}/modules/exprtk
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

SET(${PROJECT_NAME}_CPP_FILES
        src/simulator/ROSCommsDevice.cpp
        src/simulator/ROSCommsSimulator.cpp
        src/simulator/AcousticROSCommsDevice.cpp
        src/simulator/CustomROSCommsDevice.cpp
        src/simulator/AcousticCommsChannel.cpp
        src/simulator/CustomCommsChannel.cpp
        src/simulator/PacketBuilderLoader.cpp
        src/simulator/NetsimPacket.cpp
        src/simulator/CommsChannel.cpp
        src/simulator/NetsimLogFormatter.cpp
        src/simulator/NetsimTime.cpp
        src/simulator/NetsimDevice.cpp
        src/simulator/NetsimPhy.cpp
        src/simulator/NetsimRouting.cpp
)

#CHECK IF NS3 HAS ALREADY BEEN BUILT
message(STATUS "Check NS3 libraries...")
execute_process (
    COMMAND bash -c "./ns3configured.sh"
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
    OUTPUT_VARIABLE ns3configured
    ERROR_QUIET
    OUTPUT_STRIP_TRAILING_WHITESPACE
)

#https://stackoverflow.com/questions/15972898/cmake-how-to-run-a-add-custom-command-before-everything-else
#./waf configure --build-profile=debug --enable-examples --enable-tests --enable-sudo
#./waf build
#SET(WAF_EXECUTABLE ${CMAKE_CURRENT_SOURCE_DIR}/modules/ns-3-dev/waf)
if("${ns3configured}" STREQUAL "true")
    message(STATUS "Build NS3.")
    SET(WAF_EXECUTABLE ./waf)
    add_custom_target(
      ns3
      COMMAND ${WAF_EXECUTABLE} build
      WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/modules/ns-3-dev/"
    )
else()
    message(STATUS "Configure and build NS3 in ${NS3_BUILD}")
    SET(WAF_EXECUTABLE ./waf)
    add_custom_target(
      ns3
      COMMAND ${WAF_EXECUTABLE} configure --build-profile=debug --enable-examples --enable-tests -o ${NS3_BUILD}#--enable-sudo
      COMMAND ${WAF_EXECUTABLE} build
      WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/modules/ns-3-dev/"
    )
endif()

add_library(${PROJECT_NAME} SHARED ${${PROJECT_NAME}_CPP_FILES})
target_include_directories(${PROJECT_NAME} PUBLIC ${${PROJECT_NAME}_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}  ${ns3liblist} ${DCCOMMS_UTILS_GIT} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} dccomms_ros_msgs_generate_messages_cpp ns3)

#target_compile_options(pruebas PUBLIC "")
#https://cmake.org/cmake/help/v3.4/command/find_library.html
#https://stackoverflow.com/questions/31438916/cmake-cannot-find-library-using-link-directories
add_executable (example-0 ns3-scripts/example-0.cpp)
target_link_libraries(example-0 ${PROJECT_NAME})
target_include_directories(example-0 PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

add_executable (BMac ns3-scripts/BMac.cpp)
target_link_libraries(BMac ${PROJECT_NAME})
target_include_directories(BMac PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

add_executable (VBF ns3-scripts/VBF.cpp)
target_link_libraries(VBF ${PROJECT_NAME})
target_include_directories(VBF PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

add_executable (dccomms-csma ns3-scripts/dccomms-csma.cpp)
target_link_libraries(dccomms-csma ${PROJECT_NAME})
target_include_directories(dccomms-csma PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

add_executable (dccomms-wifi ns3-scripts/dccomms-wifi.cpp)
target_link_libraries(dccomms-wifi ${PROJECT_NAME})
target_include_directories(dccomms-wifi PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

add_executable (aqua-sim-mac ns3-scripts/aqua-sim-mac.cpp)
target_link_libraries(aqua-sim-mac ${PROJECT_NAME})
target_include_directories(aqua-sim-mac PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

###############################################
############ END netsim BUILD #################
###############################################

set(EXPRTK_HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/modules/exprtk/exprtk.hpp)

install(DIRECTORY 
  src/include/${PROJECT_NAME}
  ${NS3_HEADERS_DIR}
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
)
install(FILES
  ${EXPRTK_HEADERS}
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
)
install(FILES
 ${ns3liblist}
 DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
message("End ${PROJECT_NAME}")
