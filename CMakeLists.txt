cmake_minimum_required(VERSION 3.5)
project(tamagawa_imu_driver)

### Compile options
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

### Find Packages
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

set(ENABLE_AGNOCAST_VAL "$ENV{ENABLE_AGNOCAST}")
if(NOT DEFINED ENV{ENABLE_AGNOCAST} OR ENABLE_AGNOCAST_VAL STREQUAL "")
  set(ENABLE_AGNOCAST_VAL "0")
endif()

if(ENABLE_AGNOCAST_VAL STREQUAL "1")
  find_package(agnocastlib REQUIRED)
  add_compile_definitions(USE_AGNOCAST_ENABLED)
endif()

ament_auto_add_executable(tag_serial_driver
	src/tag_serial_driver.cpp
)

ament_auto_add_executable(tag_can_driver
	src/tag_can_driver.cpp
)

if(ENABLE_AGNOCAST_VAL STREQUAL "1")
  ament_target_dependencies(tag_serial_driver
    agnocastlib
  )
  ament_target_dependencies(tag_can_driver
    agnocastlib
  )
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(INSTALL_TO_SHARE
  launch
)
