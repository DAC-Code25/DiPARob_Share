cmake_minimum_required(VERSION 3.0.2)
project(sensor_fusion)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  message_filters
  message_generation
)

# Websocketpp requires boost system and thread
find_package(Boost REQUIRED COMPONENTS system thread)
find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP REQUIRED jsoncpp)
find_package(yaml-cpp REQUIRED)
if(NOT YAML_CPP_INCLUDE_DIRS AND YAML_CPP_INCLUDE_DIR)
  set(YAML_CPP_INCLUDE_DIRS ${YAML_CPP_INCLUDE_DIR})
endif()
if(NOT YAML_CPP_LIBRARIES AND YAML_CPP_LIBRARY)
  set(YAML_CPP_LIBRARIES ${YAML_CPP_LIBRARY})
endif()
if(TARGET yaml-cpp)
  get_target_property(_YAML_CPP_INC yaml-cpp INTERFACE_INCLUDE_DIRECTORIES)
  if(_YAML_CPP_INC)
    set(YAML_CPP_INCLUDE_DIRS ${YAML_CPP_INC})
  endif()
  get_target_property(_YAML_CPP_LIB yaml-cpp IMPORTED_LOCATION_RELEASE)
  if(NOT _YAML_CPP_LIB)
    get_target_property(_YAML_CPP_LIB yaml-cpp IMPORTED_LOCATION)
  endif()
  if(_YAML_CPP_LIB)
    set(YAML_CPP_LIBRARIES ${_YAML_CPP_LIB})
  endif()
endif()
if(NOT YAML_CPP_LIBRARIES)
  set(YAML_CPP_LIBRARIES yaml-cpp)
endif()
if(NOT YAML_CPP_INCLUDE_DIRS)
  set(YAML_CPP_INCLUDE_DIRS /usr/include)
endif()

find_package(Eigen3 REQUIRED)

find_package(websocketpp REQUIRED)

if(EXISTS "/usr/share/cmake/geographiclib/")
  list(APPEND CMAKE_MODULE_PATH "/usr/share/cmake/geographiclib/")
endif()
find_package(GeographicLib REQUIRED)

find_package(CURL REQUIRED)

add_message_files(
  FILES
  Rtk.msg
  Encoder.msg
  VisionMeasurement.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES sensor_fusion
  CATKIN_DEPENDS 
    roscpp 
    std_msgs 
    sensor_msgs 
    geometry_msgs 
    nav_msgs
    tf2 
    tf2_ros 
    message_filters
    message_runtime
  DEPENDS 
    JSONCPP
    EIGEN3
    yaml-cpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${CURL_INCLUDE_DIRS}
  ${JSONCPP_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
  ${YAML_CPP_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  ${GeographicLib_INCLUDE_DIRS}
  ${WEBSOCKETPP_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/rtk_parser.cpp
  src/encoder_handler.cpp
  src/ekf_fusion.cpp
  src/ukf_fusion.cpp
  src/network_sender.cpp
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(fusion_node
  src/fusion_node.cpp
)


add_dependencies(fusion_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(fusion_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${JSONCPP_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  yaml-cpp
  ${GeographicLib_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  ${WEBSOCKETPP_LIBRARIES}
  ${Boost_LIBRARIES}
  ${CURL_LIBRARIES}
)

install(TARGETS fusion_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY
  launch
  config
  rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
