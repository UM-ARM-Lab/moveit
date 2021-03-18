cmake_minimum_required(VERSION 3.1.3)
project(moveit_ros_occupancy_map_monitor)
set(MOVEIT_LIB_NAME ${PROJECT_NAME})

if(NOT "${CMAKE_CXX_STANDARD}")
  set(CMAKE_CXX_STANDARD 14)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(Boost REQUIRED thread)

if(APPLE)
  find_package(X11 REQUIRED)
endif(APPLE)

find_package(catkin REQUIRED COMPONENTS
  moveit_core
  moveit_msgs
  geometric_shapes
  pluginlib
  tf2_ros
  OPTIONAL_COMPONENTS
  pybind11_catkin
)

find_package(Eigen3 REQUIRED)
find_package(octomap REQUIRED)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${MOVEIT_LIB_NAME}
  CATKIN_DEPENDS
    moveit_core
    moveit_msgs
    geometric_shapes
    tf2_ros
  DEPENDS
    EIGEN3
    OCTOMAP
)

include_directories(include)
include_directories(SYSTEM
                    ${catkin_INCLUDE_DIRS}
                    ${Boost_INCLUDE_DIRS}
                    ${EIGEN3_INCLUDE_DIRS}
                    ${X11_INCLUDE_DIR}
                    )

add_library(${MOVEIT_LIB_NAME}
  src/occupancy_map_monitor.cpp
  src/occupancy_map_updater.cpp
  )
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
target_link_libraries(${MOVEIT_LIB_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(moveit_ros_occupancy_map_server src/occupancy_map_server.cpp)
target_link_libraries(moveit_ros_occupancy_map_server ${MOVEIT_LIB_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

catkin_python_setup()

if (pybind11_catkin_FOUND)
    set(py_libs moveit_ros_occupancy_map_monitor)
    pybind_add_module(pymoveit_ros_occupancy_map_monitor src/pyoccupancy_map_monitor.cpp)
    target_include_directories(pymoveit_ros_occupancy_map_monitor SYSTEM PUBLIC ${catkin_INCLUDE_DIRS})
    target_link_libraries(pymoveit_ros_occupancy_map_monitor PUBLIC ${py_libs} ${catkin_LIBRARIES})

    install(TARGETS pymoveit_ros_occupancy_map_monitor LIBRARY DESTINATION ${CATKIN_GLOBAL_PYTHON_DESTINATION})
endif()


install(TARGETS ${MOVEIT_LIB_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(TARGETS moveit_ros_occupancy_map_server
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})
