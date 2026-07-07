cmake_minimum_required(VERSION 3.16)
project(radarays_gz2)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(gz-common5 REQUIRED)
find_package(rmagine REQUIRED COMPONENTS core embree)

add_library(radar_sensor_system SHARED
  src/RadarSensorSystem.cpp
)

target_include_directories(radar_sensor_system PUBLIC
  include
)

target_link_libraries(radar_sensor_system
  gz-sim8::gz-sim8
  gz-plugin2::gz-plugin2
  gz-common5::gz-common5
  rmagine::core
  rmagine::embree
)

ament_target_dependencies(radar_sensor_system
  rclcpp
  sensor_msgs
)

install(TARGETS radar_sensor_system
  LIBRARY DESTINATION lib
)

ament_package()
