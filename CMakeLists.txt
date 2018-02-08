cmake_minimum_required(VERSION 2.8.3)
project(leddar)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  roscpp
  sensor_msgs
  std_msgs
)

find_package(Qt5Core REQUIRED)

generate_dynamic_reconfigure_options(cfg/Scan.cfg)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS sensor_msgs std_msgs
)

link_directories($ENV{LEDDAR_LIB_DIR})
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} src/${PROJECT_NAME}.cpp)
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} LeddarTech Leddar LeddarC)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
