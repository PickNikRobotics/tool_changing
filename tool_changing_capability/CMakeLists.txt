cmake_minimum_required(VERSION 3.10.2)
project(tool_changing_capability)

# Common cmake code applied to all moveit packages
find_package(moveit_common REQUIRED)
moveit_package()

set(THIS_PACKAGE_INCLUDE_DEPENDS
    ament_cmake
    rclcpp
    moveit_core
    moveit_ros_move_group
    tool_changing_capability_msgs
    moveit_ros_planning
    pluginlib
)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_library(moveit_move_group_tool_changing_capability SHARED
    src/tool_changing_capability.cpp
    src/tool_changing_capability.hpp)
ament_target_dependencies(moveit_move_group_tool_changing_capability ${THIS_PACKAGE_INCLUDE_DEPENDS})

add_library(tool_changing_planning_request_adapter SHARED
    src/tool_changing_planning_request_adapter.cpp)
ament_target_dependencies(tool_changing_planning_request_adapter ${THIS_PACKAGE_INCLUDE_DEPENDS})

pluginlib_export_plugin_description_file(moveit_ros_move_group tool_changing_capability_plugin_description.xml)
pluginlib_export_plugin_description_file(moveit_core tool_changing_planning_request_adapter_plugin_description.xml)

install(
    TARGETS
    moveit_move_group_tool_changing_capability tool_changing_planning_request_adapter
    EXPORT ${PROJECT_NAME}Targets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
