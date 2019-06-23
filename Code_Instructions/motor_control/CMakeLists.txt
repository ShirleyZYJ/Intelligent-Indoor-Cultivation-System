cmake_minimum_required(VERSION 2.8.3)
project(motor_control)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

include_directories(${PROJECT_SOURCE_DIR}
                    /usr/local/include
                    )

find_package(catkin REQUIRED COMPONENTS
             apriltags_ros
             cmake_modules
             roscpp
             geometry_msgs
             nav_msgs
             std_msgs
)

find_package(Threads REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)

add_definitions("-std=c++11 -Wall")

catkin_package(
  INCLUDE_DIRS
  LIBRARIES motor_control
  CATKIN_DEPENDS apriltags_ros roscpp geometry_msgs nav_msgs std_msgs message_runtime
  DEPENDS Eigen3 BOOST
)


include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_library(motor_control
            src/dualmotor.cpp
            src/singlemotor.cpp
            src/modbus.cpp
            src/crc16.cpp
            src/motor_control.cpp
            )

target_link_libraries(motor_control
  ${catkin_LIBRARIES}
  ${Eigen3_LIBRARIES}
)

add_executable(motor_control_node src/motor_control_node.cpp)

target_link_libraries(motor_control_node
                      ${catkin_LIBRARIES}
                      ${CMAKE_THREAD_LIBS_INIT} 
                      motor_control
)

install(TARGETS motor_control motor_control_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/motor_control_node
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
