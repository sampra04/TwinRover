cmake_minimum_required(VERSION 3.8)
project(twinrover_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf launch config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/twist_to_stamped.py
  scripts/stall_guard.py
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
