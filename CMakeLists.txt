cmake_minimum_required(VERSION 3.2)
project(CamSimulate_Display)


set(CMAKE_PREFIX_PATH "/home/zzl/opencv420/build/install/") # 设置正确的OpenCV路径
find_package(OpenCV 4 REQUIRED)
message(STATUS "OpenCV library status:")
message(STATUS "   OpenCV Version: ${OpenCV_VERSION}" )
find_package(Pangolin)
find_package(Eigen3)


set(CMAKE_CXX_STANDARD 14)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${Pangolin_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})


add_executable(CamSimulate main.cpp)
target_link_libraries(CamSimulate ${OpenCV_LIBS} ${Pangolin_LIBRARIES} )#${PCL_LIBRARIES})




