cmake_minimum_required(VERSION 3.16)
project(igus_axis_driver LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()


## COMPILE
add_library(
  igus_axis_driver
  SHARED
  hardware/singlebot.cpp
  hardware/D1.cpp

)
target_compile_features(igus_axis_driver PUBLIC cxx_std_17)
target_include_directories(igus_axis_driver PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
$<INSTALL_INTERFACE:include/igus_axis_driver>
)
ament_target_dependencies(
  igus_axis_driver PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "igus_axis_driver_BUILDING_DLL")

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface igus_axis_driver.xml)

# INSTALL
install(
  DIRECTORY hardware/include/
  DESTINATION include/igus_axis_driver
)

install(
  DIRECTORY  rviz 
  DESTINATION share/${PROJECT_NAME}
)


install(
  DIRECTORY bringup/launch bringup/config
  DESTINATION share/igus_axis_driver
)
install(TARGETS igus_axis_driver
  EXPORT export_igus_axis_driver
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
endif()

## EXPORTS
ament_export_targets(export_igus_axis_driver HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
