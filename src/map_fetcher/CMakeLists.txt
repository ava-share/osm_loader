cmake_minimum_required(VERSION 3.5)
project(map_fetcher)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(map_fetcher)

install(
  PROGRAMS
    map_fetcher/map_fetcher_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
