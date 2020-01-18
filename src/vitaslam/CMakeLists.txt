cmake_minimum_required(VERSION 2.8.3)
project(vitaslam)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  roscpp
  sensor_msgs
  cv_bridge
  nav_msgs
  tf
  visualization_msgs
  image_transport
  cmake_modules
)

## System dependencies are found with CMake's conventions
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
find_package(Boost REQUIRED COMPONENTS serialization)
# Packages required for drawing
find_package(Irrlicht REQUIRED)
find_package(OpenGL REQUIRED)


## Generate messages in the 'msg' folder
add_message_files(
   FILES
   TopologicalAction.msg
   TopologicalEdge.msg
   TopologicalMap.msg
   TopologicalNode.msg
)
## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

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
    CATKIN_DEPENDS message_runtime geometry_msgs std_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
add_library(vitaslam src/vitaslam/experience_map.cpp src/vitaslam/posecell_network.cpp src/vitaslam/local_template_match.cpp src/vitaslam/posecells.cpp src/vitaslam/preprocess.cpp)

# uncomment is you don't have irrlicht installed
add_definitions("-DHAVE_IRRLICHT")

add_executable(vitaslam_em src/main_em.cpp)
target_link_libraries(vitaslam_em
  ${catkin_LIBRARIES}
  vitaslam
  ${IRRLICHT_LIBRARIES}
  ${OPENGL_LIBRARIES})
add_executable(vitaslam_pc src/main_pc.cpp)
target_link_libraries(vitaslam_pc
  ${catkin_LIBRARIES}
  vitaslam
  ${IRRLICHT_LIBRARIES}
  ${OPENGL_LIBRARIES})
add_executable(vitaslam_vita src/main_vita.cpp)
target_link_libraries(vitaslam_vita
  ${catkin_LIBRARIES}
  vitaslam
  ${UUID_LIBRARIES}
  ${IRRLICHT_LIBRARIES}
  ${OPENGL_LIBRARIES}
  ${OpenCV_LIBRARIES})

# More stuff for drawing using irrlicht, configure media path and so on
#config files for devel
set(MEDIA_PATH ${CMAKE_CURRENT_SOURCE_DIR}/src/media) #devel use the files in the source dir
file(GLOB infiles "config/*.in")
foreach(infile ${infiles})
  #MESSAGE(STATUS "Process file: ${infile}")

  # Generate output file name
  string(REGEX REPLACE ".in\$" "" outfile "${infile}")
  get_filename_component(outfile "${outfile}" NAME)
  set(outfile "${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/config/${outfile}")
  #MESSAGE(STATUS "Output file: ${outfile}")

  configure_file("${infile}" "${outfile}")
endforeach()

#config files for install
set(MEDIA_PATH ${CMAKE_INSTALL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/media) #install use the installed media files
file(GLOB infiles "config/*.in")
foreach(infile ${infiles})
    #MESSAGE(STATUS "Process file: ${infile}")

  # Generate output file name
  string(REGEX REPLACE ".in\$" "" outfile "${infile}")
  get_filename_component(outfile "${outfile}" NAME)
  set(outfile "${CMAKE_CURRENT_BINARY_DIR}/config/${outfile}")
  #MESSAGE(STATUS "Output file: ${outfile}")

  configure_file("${infile}" "${outfile}")
endforeach()

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(vitaslam_em vitaslam_ros_generated_messages_cpp)
add_dependencies(vitaslam_pc vitaslam_ros_generated_messages_cpp)
add_dependencies(vitaslam_vita vitaslam_ros_generated_messages_cpp)

install(TARGETS vitaslam vitaslam_em vitaslam_pc
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
 
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
