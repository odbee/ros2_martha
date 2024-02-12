cmake_minimum_required(VERSION 3.8)
project(mv2_com)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# set(THIS_PACKAGE_INCLUDE_DEPENDS
#   ament_cmake
#   Eigen3
#   moveit_core
#   moveit_msgs
#   moveit_ros_planning
#   moveit_ros_planning_interface
#   # moveit_task_constructor_core
#   moveit_visual_tools
#   pluginlib
#   rclcpp
#   rclcpp_action
#   rclcpp_components

#   rviz_visual_tools
#   tf2_geometry_msgs
#   tf2_ros
#   geometry_msgs
#   mv2_com_interfaces
#   example_interfaces
# )

# foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
#   find_package(${Dependency} REQUIRED)
# endforeach()



# add_executable(mv2_com_server src/mv2_com_server.cpp)
# add_executable(ur_mv2_com_server src/ur_mv2_com_server.cpp)

# add_executable(mg_com_server src/mg_com_server.cpp)

# target_include_directories(mv2_com_server PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)

# target_include_directories(mv2_com_server PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)


# target_include_directories(mg_com_server PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
 
# target_compile_features(mv2_com_server PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
# target_compile_features(mg_com_server PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
# target_compile_features(ur_mv2_com_server PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

# ament_target_dependencies(
#   mv2_com_server

#   ${THIS_PACKAGE_INCLUDE_DEPENDS}
# )

# ament_target_dependencies(
#   ur_mv2_com_server

#   ${THIS_PACKAGE_INCLUDE_DEPENDS}
# )


# ament_target_dependencies(
#   mg_com_server
#   ${THIS_PACKAGE_INCLUDE_DEPENDS}
# )



# install(TARGETS
#     mv2_com_server
#     mg_com_server
#   DESTINATION lib/${PROJECT_NAME})


# install(DIRECTORY launch
#   DESTINATION share/${PROJECT_NAME}
# )

# install(DIRECTORY config
#   DESTINATION share/${PROJECT_NAME}
# )


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
