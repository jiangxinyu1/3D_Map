cmake_minimum_required(VERSION 2.8.3)
project(skiMap_node)


set( CMAKE_BUILD_TYPE "Release")
set( CMAKE)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Ofast -mavx -mfma ")

#  -Ofast -mavx -mfma

set(Thirdparty_DIR ~/cleanRobot/thirdparty)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${Thirdparty_DIR}/CMakeModules")
set( EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)


# -Ofast -mavx -mfma

find_package(lcm REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
    ${INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}/src/include
    ${PROJECT_SOURCE_DIR}/src/include/skimap_ros
    ${PROJECT_SOURCE_DIR}/src/include/skimap
    ${PROJECT_SOURCE_DIR}/src/include/slamdunk
    ${PROJECT_SOURCE_DIR}/src/include/tf
    ${PROJECT_SOURCE_DIR}/src/include/slamdunk_extension
    ${Thirdparty_DIR}/include
    ${Thirdparty_DIR}/include/lcm_msgs
    ${Thirdparty_DIR}/include/lcm_msgs/lcm_std_msgs
    ${Thirdparty_DIR}/include/lcm_msgs/lcm_sensor_msgs
    ${Thirdparty_DIR}/include/lcm_msgs/lcm_geometry_msgs
    ${Thirdparty_DIR}/include
    ${Thirdparty_DIR}/include/opencv2
    ${LCM_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)


FILE(
  GLOB
  SRC_FILES
  ${PROJECT_SOURCE_DIR}/src/lcmHandler.cpp
  ${PROJECT_SOURCE_DIR}/src/tool.cpp
  ${PROJECT_SOURCE_DIR}/src/main.cpp
)


add_executable(skiMap_node ${SRC_FILES})
target_link_libraries(skiMap_node ${LCM_LIBRARY} ${OpenCV_LIBS})