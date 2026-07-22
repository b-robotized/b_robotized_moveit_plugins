cmake_minimum_required(VERSION 3.22)
project(cubic_spline_smoothing_adapter)

find_package(ament_cmake REQUIRED)
find_package(moveit_core REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)

add_library(cubic_spline_smoothing_adapter SHARED src/cubic_spline_smoothing_adapter.cpp)
target_include_directories(cubic_spline_smoothing_adapter PUBLIC include)
ament_target_dependencies(cubic_spline_smoothing_adapter 
    moveit_core 
    pluginlib 
    rclcpp
)

# Export the plugin description
pluginlib_export_plugin_description_file(moveit_core cubic_spline_smoothing_adapter_plugin.xml)

install(
  TARGETS cubic_spline_smoothing_adapter
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include)

ament_package()