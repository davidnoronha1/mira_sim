cmake_minimum_required(VERSION 3.8)
project(tacc_sim)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config models worlds DESTINATION share/${PROJECT_NAME})

ament_package()
