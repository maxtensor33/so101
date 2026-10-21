cmake_minimum_required(VERSION 3.16)
project(so101_mtc_pick_place)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(moveit_task_constructor_msgs REQUIRED)
find_package(moveit_ros_move_group REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(mujoco_ros2_control_plugins REQUIRED)
find_package(std_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  moveit_task_constructor_core
  moveit_task_constructor_msgs
  moveit_ros_move_group
  moveit_ros_planning_interface
  rclcpp
  pluginlib
  mujoco_ros2_control_plugins
  std_msgs
  control_msgs
  geometry_msgs
  moveit_msgs
  sensor_msgs
  tf2_eigen
  tf2_geometry_msgs
)

# ── MTC pick and place 节点 ─────────────────────────────────────────────────
add_executable(mtc_pick_place_node src/mtc_pick_place_node.cpp)
ament_target_dependencies(mtc_pick_place_node ${THIS_PACKAGE_INCLUDE_DEPENDS})
install(TARGETS mtc_pick_place_node DESTINATION lib/${PROJECT_NAME})

# ── mujoco 抓取 plugin (kinematic attach) ───────────────────────────────────
add_library(grasp_attach_plugin SHARED src/grasp_attach_plugin.cpp)
# pluginlib 基类与 mujoco 由依赖包传递链接
ament_target_dependencies(grasp_attach_plugin ${THIS_PACKAGE_INCLUDE_DEPENDS})
pluginlib_export_plugin_description_file(mujoco_ros2_control_plugins grasp_attach_plugin.xml)
install(TARGETS grasp_attach_plugin
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)

install(DIRECTORY launch rviz config DESTINATION share/${PROJECT_NAME})
install(FILES grasp_attach_plugin.xml DESTINATION share/${PROJECT_NAME})

ament_package()
