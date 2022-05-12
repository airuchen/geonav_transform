cmake_minimum_required(VERSION 2.8.3)
project(geonav_transform)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geographic_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  message_generation
  tf
  navgraph
)

# Attempt to find Eigen using its own CMake module.
# If that fails, fall back to cmake_modules package.
#find_package(Eigen3)
find_package(Eigen)
set(EIGEN_PACKAGE EIGEN3)
if(NOT EIGEN3_FOUND)
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
  set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})
  set(EIGEN_PACKAGE Eigen)
endif()

add_definitions(-DEIGEN_NO_DEBUG -DEIGEN_MPL2_ONLY)

add_service_files(
   FILES
   RequestGPS.srv
)

generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES geonav_transform
   CATKIN_DEPENDS 
    roscpp
    cmake_modules
    geographic_msgs
    geometry_msgs
    nav_msgs
    sensor_msgs
    std_msgs
    tf2
    tf2_geometry_msgs
    tf2_ros
   DEPENDS ${EIGEN_PACKAGE}
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})

## Declare a C++ library
add_library(geonav_transform
   src/geonav_transform.cpp
   src/geonav_utilities.cpp
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure

## Declare a C++ executable
add_executable(geonav_transform_node src/geonav_transform_node.cpp)
add_dependencies(geonav_transform_node geonav_transform_generate_messages_cpp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(geonav_transform_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(geonav_transform
   ${catkin_LIBRARIES} 
   ${EIGEN3_LIBRARIES}
)
target_link_libraries(geonav_transform_node geonav_transform
   ${catkin_LIBRARIES} 
 )

add_executable(gps_goal_transform src/gps_goal_transform.cpp)
target_link_libraries(gps_goal_transform ${catkin_LIBRARIES})
add_dependencies(gps_goal_transform geonav_transform_generate_messages_cpp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})