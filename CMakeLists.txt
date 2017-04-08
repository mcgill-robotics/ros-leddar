cmake_minimum_required(VERSION 2.8.3)
project(leddar)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  dynamic_reconfigure
)

generate_dynamic_reconfigure_options(cfg/Scan.cfg)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

link_directories($ENV{LEDDAR_LIB_DIR})

add_executable(leddar src/leddar.cpp)
add_dependencies(leddar ${PROJECT_NAME}_gencfg)
target_link_libraries(leddar ${catkin_LIBRARIES} LeddarTech Leddar LeddarC)

