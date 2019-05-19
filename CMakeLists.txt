cmake_minimum_required(VERSION 2.6)

project(3DScanCap)

SET(CMAKE_BUILD_TYPE Release)

find_package(OpenCV 3 REQUIRED)
find_package(freenect2 REQUIRED)
find_package(PCL REQUIRED COMPONENT common io )

include_directories(${OpenCV_INCLUDE_DIR})
include_directories(${freenect2_INCLUDE_DIR})
include_directories( ${PCL_INCLUDE_DIRS} )

add_definitions( ${PCL_DEFINITIONS} )

add_executable(3DScanCap 3DScanCap.cpp)
target_link_libraries(3DScanCap opencv_core opencv_highgui ${freenect2_LIBRARIES} ${PCL_LIBRARIES})
