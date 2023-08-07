cmake_minimum_required(VERSION 3.5)
project(dex_controller)

find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_core REQUIRED)
find_package(pluginlib REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

nav2_package()
set(CMAKE_CXX_STANDARD 17)

# Temporary disable unused-parameter error/warning for the dummy controller.
# Please remove this line when done developing =)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-error=unused-parameter -Wno-unused-parameter")

include_directories(
  include
)
set(library_name dex_controller)
set(dependencies
    rclcpp
    rclcpp_lifecycle
    geometry_msgs
    pluginlib
    nav_msgs
    nav2_core
)

add_library(dex_controller SHARED
    src/dex_controller.cpp)
ament_target_dependencies(dex_controller
    rclcpp
    geometry_msgs
    pluginlib
    nav_msgs
    nav2_core
)

install(TARGETS dex_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(dex_controller)
ament_export_dependencies(${dependencies})

pluginlib_export_plugin_description_file(nav2_core dex_controller.xml)

ament_package()
