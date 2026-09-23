cmake_minimum_required(VERSION 3.8)
project(medicine_delivery_sim)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/medicine_delivery.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  worlds
  maps
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
