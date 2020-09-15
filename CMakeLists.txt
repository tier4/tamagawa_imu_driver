cmake_minimum_required(VERSION 3.0.2)
project(tamagawa_imu_driver)

find_package(catkin REQUIRED COMPONENTS
  can_msgs
  diagnostic_msgs
  roscpp
  sensor_msgs
)

find_package(fmt REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    can_msgs
    diagnostic_msgs
    diagnostic_updater
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(tag_serial_driver
  src/tag_serial_driver_node.cpp
  src/tag_serial_driver.cpp
)

target_link_libraries(tag_serial_driver ${catkin_LIBRARIES} fmt)

add_executable(tag_can_driver src/tag_can_driver.cpp)
target_link_libraries(tag_can_driver ${catkin_LIBRARIES})

install(TARGETS
  tag_serial_driver
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
