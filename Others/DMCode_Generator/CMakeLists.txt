# cmake version mini requirement
CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

add_compile_options(-std=c++11)

# project name
project(CodeGenerator)

# find the opencv Package
find_package(OpenCV REQUIRED)

include_directories(include 
                      include/DataMatrix
                      ${OpenCV_INCLUDE_DIRs})

# add executable file
add_executable(CodeGenerator
                  src/CodeGenerator.cpp 
                  src/dataMatrixEncoder.cpp 
                  src/dataMatrixDecoder.cpp 
                  src/ReedSolomon.cpp 
                  src/ReedSolomonEncoder.cpp 
                  src/ReedSolomonDecoder.cpp 
                  src/DrawCode2D.cpp
)

# target link library to exe
 target_link_libraries(CodeGenerator 
                         ${OpenCV_LIBRARIES}
)
