cmake_minimum_required (VERSION 2.6)
project(ht_vision_node)

set(CMAKE_CXX_FLAGS "-std=c++11 -O3")

include_directories(${catkin_INCLUDE_DIRS}
                    /usr/local/include
                    )

find_package(catkin REQUIRED COMPONENTS
             cmake_modules
             roscpp
             sensor_msgs
             std_msgs
             apriltags_ros
            )

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)

catkin_package(
  INCLUDE_DIRS
  LIBRARIES ht_vision
  CATKIN_DEPENDS apriltags_ros roscpp geometry_msgs  std_msgs message_runtime
  DEPENDS Eigen3 BOOST
)


include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_library(ht_vision
            src/dualmotor.cpp
            src/singlemotor.cpp
            src/modbus.cpp
            src/crc16.cpp
            src/ht_vision.cpp
            src/tag_correction.cpp
            )

target_link_libraries(ht_vision
  ${catkin_LIBRARIES}
  ${Eigen3_LIBRARIES}
)

add_executable(ht_vision_node src/ht_vision.cpp)

target_link_libraries(ht_vision_node
  ${catkin_LIBRARIES}
  ${Eigen3_LIBRARIES}
  ht_vision
)

add_executable(tag_correction_node src/tag_correction_node.cpp)

target_link_libraries(tag_correction_node
  ${catkin_LIBRARIES}
  ${Eigen3_LIBRARIES}
  ht_vision
)

install(TARGETS ht_vision_node ht_vision tag_correction_node 
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/ht_vision_node 
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY include/tag_correction_node 
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)



