cmake_minimum_required(VERSION 3.5.1)
project(dccomms_ros CXX)

SET(DISABLE_GIT_HASH ON CACHE BOOL "Disables git hash suffix on libraries")

#It seams that from Ubuntu 17.10 the linker flag --disable-new-dtags is needed
#for the executable to be able to find absolute linked shared libraries with rpath.
#https://stackoverflow.com/questions/47117443/dynamic-linking-with-rpath-not-working-under-ubuntu-17-10
#SET(CMAKE_EXE_LINKER_FLAGS "-Wl,--disable-new-dtags")

set(CMAKE_VERBOSE_MAKEFILE ON)

SET(MODULES_DIR "modules" )

## Find libraries
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(dccomms_ros_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(class_loader REQUIRED)

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
find_package(Git REQUIRED)
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

#CHECK IF NS3 HAS ALREADY BEEN BUILT
message(STATUS "Check NS3 libraries...")
execute_process (
    COMMAND bash -c "./ns3configured.sh"
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
    OUTPUT_VARIABLE ns3configured
    ERROR_QUIET
    OUTPUT_STRIP_TRAILING_WHITESPACE
)

SET(NS3_BUILD ${CMAKE_CURRENT_SOURCE_DIR}/modules/ns-3-dev/build)
SET(NS3_INCLUDE_PATH ${NS3_BUILD})
SET(NS3_HEADERS_DIR ${NS3_BUILD}/ns3)

message("NS3 build path: ${NS3_BUILD}")
message("NS3 include path: ${NS3_INCLUDE_PATH}")
message("NS3 headers dir: ${NS3_HEADERS_DIR}")
execute_process(
  COMMAND mkdir -p ${NS3_BUILD}
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)

#https://stackoverflow.com/questions/15972898/cmake-how-to-run-a-add-custom-command-before-everything-else
#./waf configure --build-profile=debug --enable-examples --enable-tests --enable-sudo
#./waf build
#SET(WAF_EXECUTABLE ${CMAKE_CURRENT_SOURCE_DIR}/modules/ns-3-dev/waf)
if("${ns3configured}" STREQUAL "true")
    message(STATUS "Build NS3.")
    SET(WAF_EXECUTABLE ./waf)
    execute_process(
      COMMAND ${WAF_EXECUTABLE} build
      WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/modules/ns-3-dev/"
    )
else()
    message(STATUS "Configure and build NS3 in ${NS3_BUILD}")
    SET(WAF_EXECUTABLE ./waf)
    execute_process(
      COMMAND ${WAF_EXECUTABLE} configure --build-profile=debug --enable-examples --enable-tests --disable-python -o ${NS3_BUILD}#--enable-sudo
      WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/modules/ns-3-dev/"
    )
    execute_process(
      COMMAND ${WAF_EXECUTABLE} build
      WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/modules/ns-3-dev/"
    )
endif()

SET(NS3_LIBS
    ns3-dev-antenna-debug                ns3-dev-flow-monitor-test-debug      ns3-dev-propagation-debug
    ns3-dev-antenna-test-debug           ns3-dev-internet-apps-debug          ns3-dev-propagation-test-debug
    ns3-dev-aodv-debug                   ns3-dev-internet-apps-test-debug     ns3-dev-sixlowpan-debug
    ns3-dev-aodv-test-debug              ns3-dev-internet-debug               ns3-dev-sixlowpan-test-debug
    ns3-dev-applications-debug           ns3-dev-internet-test-debug          ns3-dev-spectrum-debug
    ns3-dev-applications-test-debug      ns3-dev-lr-wpan-debug                ns3-dev-spectrum-test-debug
    ns3-dev-aqua-sim-ng-debug            ns3-dev-lr-wpan-test-debug           ns3-dev-stats-debug
    ns3-dev-aqua-sim-ng-test-debug       ns3-dev-lte-debug                    ns3-dev-stats-test-debug
    ns3-dev-bridge-debug                 ns3-dev-lte-test-debug               ns3-dev-tap-bridge-debug
    ns3-dev-buildings-debug              ns3-dev-mesh-debug                   ns3-dev-test-debug
    ns3-dev-buildings-test-debug         ns3-dev-mesh-test-debug              ns3-dev-test-test-debug
    ns3-dev-config-store-debug           ns3-dev-mobility-debug               ns3-dev-topology-read-debug
    ns3-dev-core-debug                   ns3-dev-mobility-test-debug          ns3-dev-topology-read-test-debug
    ns3-dev-core-test-debug
    #ns3-dev-mpi-debug
    ns3-dev-traffic-control-debug
    ns3-dev-csma-debug                   ns3-dev-netanim-debug                ns3-dev-traffic-control-test-debug
    ns3-dev-csma-layout-debug            ns3-dev-netanim-test-debug           ns3-dev-uan-debug
    ns3-dev-dsdv-debug                   ns3-dev-network-debug                ns3-dev-uan-test-debug
    ns3-dev-dsdv-test-debug              ns3-dev-network-test-debug           ns3-dev-virtual-net-device-debug
    ns3-dev-dsr-debug                    ns3-dev-nix-vector-routing-debug     ns3-dev-wave-debug
    ns3-dev-dsr-test-debug               ns3-dev-olsr-debug                   ns3-dev-wave-test-debug
    ns3-dev-energy-debug                 ns3-dev-olsr-test-debug              ns3-dev-wifi-debug
    ns3-dev-energy-test-debug            ns3-dev-point-to-point-debug         ns3-dev-wifi-test-debug
    ns3-dev-fd-net-device-debug          ns3-dev-point-to-point-layout-debug  ns3-dev-wimax-debug
    ns3-dev-flow-monitor-debug           ns3-dev-point-to-point-test-debug    ns3-dev-wimax-test-debug
)

#Catkin do not export well relative paths to libraries: https://github.com/ros/catkin/issues/535
#link_directories(${NS3_BUILD})
#So it's better to specify the full path to each .so if the the *.so are not in the common directories (see ldconfig)
message("NS3 modules: ${NS3_LIBS}")
FOREACH(lib ${NS3_LIBS})
	find_library(NS3LIB_${lib} NAMES "${lib}" PATHS "${NS3_BUILD}/lib")
	if(NOT NS3LIB_${lib})
		message(FATAL_ERROR "NS3 LIBRARY ${lib} NOT FOUND")
	else()
		message(STATUS "#####################  NS3 LIBRARY FOUND: ${NS3LIB_${lib}}")
	endif()
ENDFOREACH()


SET(${PROJECT_NAME}_INCLUDE_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}/src/include
    ${NS3_INCLUDE_PATH}
    ${CMAKE_CURRENT_SOURCE_DIR}/modules/exprtk
    ${CMAKE_CURRENT_SOURCE_DIR}/modules/dccomms_utils/src/lib/include
    ${CMAKE_CURRENT_SOURCE_DIR}/modules/dccomms_utils/modules/dccomms_packets/src/include
    ${CMAKE_CURRENT_SOURCE_DIR}/modules/dccomms_utils/modules/cxxopts/include
    ${CMAKE_CURRENT_SOURCE_DIR}/modules/dccomms_utils/modules/dccomms_packets/modules/dccomms/include
    ${CMAKE_CURRENT_SOURCE_DIR}/modules/dccomms_utils/modules/dccomms_packets/modules/dccomms/deps/cpplogging/includes
    ${CMAKE_CURRENT_SOURCE_DIR}/modules/dccomms_utils/modules/dccomms_packets/modules/dccomms/deps/cpputils/src/include
    )


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


add_compile_options(-std=c++17)
SET(${PROJECT_NAME}_CXX_COMPILE_FLAGS "-fPIC -std=c++17 ")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${${PROJECT_NAME}_CXX_COMPILE_FLAGS}")


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
        # src/simulator/NetsimLogFormatter.cpp
        src/simulator/NetsimTime.cpp
        src/simulator/NetsimDevice.cpp
        src/simulator/NetsimPhy.cpp
        src/simulator/NetsimRouting.cpp
)

#https://answers.ros.org/question/97606/export-a-prebuilt-library-in-catkin/
#https://github.com/ros-drivers/prosilica_gige_sdk/blob/hydro-devel/CMakeLists.txt

FOREACH(lib ${NS3_LIBS})
	set(ns3target ${lib})
	message(STATUS "############# NS3 LIBRARY ${NS3LIB_${lib}} ----> ${ns3target}")
	add_library(${ns3target} SHARED EXCLUDE_FROM_ALL ${NS3LIB_${lib}})
	#add_custom_command(TARGET ${ns3target} POST_BUILD COMMAND cp ${NS3LIB_${lib}} ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/ )

	set_target_properties(${ns3target} PROPERTIES LINKER_LANGUAGE CXX )
	install(
		FILES ${NS3LIB_${lib}}
   		DESTINATION lib
	)
ENDFOREACH()



include_directories( 
  ${${PROJECT_NAME}_INCLUDE_DIRS}
  ${dccomms_ros_msgs_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}
  )

add_library(${PROJECT_NAME} SHARED ${${PROJECT_NAME}_CPP_FILES})
target_link_libraries(${PROJECT_NAME} PUBLIC ${NS3_LIBS} ${DCCOMMS_UTILS_GIT}  
  ${rclcpp_LIBRARIES}
  ${tf2_ros_LIBRARIES}
  ${class_loader_LIBRARIES}
  ${dccomms_ros_msgs_LIBRARIES}
)

ament_export_include_directories(include ${${PROJECT_NAME}_INCLUDE_DIRS})
ament_export_dependencies(dccomms_ros_msgs)

###############################################
############ END netsim BUILD #################
###############################################

set(EXPRTK_HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/modules/exprtk/exprtk.hpp)

install(DIRECTORY 
  src/include/${PROJECT_NAME}
  ${NS3_HEADERS_DIR}
  DESTINATION include/${PROJECT_NAME}
)
install(FILES
  ${EXPRTK_HEADERS}
  DESTINATION include/${PROJECT_NAME}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_libraries(${PROJECT_NAME} 
    ${NS3_LIBS}
    ${DCCOMMS_UTILS_GIT} 
    cpputils 
    cpplogging 
    dccomms
    dccomms_packets
)

ament_package()
