cmake_minimum_required(VERSION 2.8.3)
project(xrsolve_ik_kinematics_plugin)

if(CMAKE_COMPILER_IS_GNUCXX OR ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang"))
    set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")
endif()

find_package(catkin REQUIRED
  COMPONENTS
    moveit_core
    pluginlib
    roscpp
    tf_conversions
    xrsolve_ik_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    moveit_core
    pluginlib
    roscpp
    tf_conversions
    xrsolve_ik_lib
)

set(XRSOLVE_IK_LIBRARY_NAME xrsolve_ik_kinematics_plugin)

add_library(${XRSOLVE_IK_LIBRARY_NAME} src/xrsolve_ik_kinematics_plugin.cpp)
target_link_libraries(${XRSOLVE_IK_LIBRARY_NAME} ${catkin_LIBRARIES})

install(DIRECTORY include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
)

install(TARGETS ${XRSOLVE_IK_LIBRARY_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(
  FILES
    xrsolve_ik_kinematics_description.xml
  DESTINATION
    ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
