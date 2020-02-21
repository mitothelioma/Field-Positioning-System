cmake_minimum_required(VERSION 3.1)
# Enable C++11
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
SET(OpenCV_DIR ../opencv/installation/OpenCV-master/lib/cmake/opencv4)

project (basicballdetector)
find_package(OpenCV 4 REQUIRED)
find_package(Threads REQUIRED)
find_package(realsense2 REQUIRED)

if (WIN32)
    find_package(czmq REQUIRED)
    find_package(zeromq REQUIRED)

    # Remove libzmq-static from czmq
    get_target_property(czmq_LIBRARIES czmq INTERFACE_LINK_LIBRARIES)
    LIST(REMOVE_ITEM czmq_LIBRARIES libzmq-static)
    set_property(TARGET czmq PROPERTY INTERFACE_LINK_LIBRARIES ${czmq_LIBRARIES})
endif()

set(Boost_USE_MULTITHREADED      ON)
set(Boost_USE_STATIC_RUNTIME     OFF)
find_package(Boost 1.65.0 REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS}/include)

add_executable (basiccolordetector BasicColorDetector.cpp)
target_link_libraries(basiccolordetector czmq ${OpenCV_LIBS} realsense2::realsense2)
if (WIN32)
     target_link_libraries(basiccolordetector libzmq)
else()
    target_link_libraries(basiccolordetector zmq)
endif()