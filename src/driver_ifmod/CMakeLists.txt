cmake_minimum_required(VERSION 2.8.3)
project(driver_ifmod)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
if(${CMAKE_CXX_COMPILER} STREQUAL "/usr/bin/c++")
  add_compile_options(-fext-numeric-literals)
elseif(${CMAKE_CXX_COMPILER} STREQUAL "/usr/bin/clang++")
  MESSAGE(STATUS "complier is ${CMAKE_CXX_COMPILER}, so DO NOT add compile options: -fext-numeric-literals" )
endif()

find_package(catkin REQUIRED COMPONENTS
   pcl_conversions
   pcl_msgs
   pcl_ros
   std_msgs
   visualization_msgs
   message_generation
)

add_message_files(
  FILES
  calibration_msg.msg
  CalibResult.msg
  CrashPredictorResult.msg
  logic_msg.msg
  objectData_t_ch8_msg.msg
  ObjectList.msg

  )

generate_messages(
    DEPENDENCIES
    std_msgs
 )




include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  src
)

add_executable(driver_ifmod
  src/driver_ifmod_node.cpp
  src/camera_points.cpp
  src/UDPGet.cpp
  src/Channels/OD_CH08_1_4.cpp
  src/Channels/OD_CH14_1_5.cpp
  src/Channels/OD_CH20_1_4.cpp
  src/Channels/OD_CH256_1_6.cpp)
target_link_libraries(driver_ifmod ${catkin_LIBRARIES})
add_dependencies(driver_ifmod ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

install(TARGETS driver_ifmod
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY
        launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
      )
install(DIRECTORY
        params/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/params
      )