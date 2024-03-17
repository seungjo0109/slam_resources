cmake_minimum_required(VERSION 3.0)
project(kitti_rerun_visualization)

# C++17
set(CMAKE_CXX_STANDARD 17)

# Download rerun SDK
include(FetchContent)
FetchContent_Declare(rerun_sdk URL
    https://github.com/rerun-io/rerun/releases/latest/download/rerun_cpp_sdk.zip)
FetchContent_MakeAvailable(rerun_sdk)

# Download the kitti dataset

# Build the hello_rerun example
add_executable(hello_rerun hello_rerun.cpp)
target_link_libraries(hello_rerun PRIVATE rerun_sdk)

# Build the kitti_rerun_visualization example
add_executable(kitti_rerun_visualization kitti_rerun_visualization.cpp)
target_link_libraries(kitti_rerun_visualization PRIVATE rerun_sdk)
