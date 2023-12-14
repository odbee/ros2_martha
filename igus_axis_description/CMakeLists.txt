cmake_minimum_required(VERSION 3.5)
project(igus_axis_description)

find_package(ament_cmake REQUIRED)

# install(
#   DIRECTORY rrbot/urdf rrbot/rviz
#   DESTINATION share/${PROJECT_NAME}/rrbot
# )

install(
  DIRECTORY urdf rviz launch
  DESTINATION share/${PROJECT_NAME}
)



ament_package()
