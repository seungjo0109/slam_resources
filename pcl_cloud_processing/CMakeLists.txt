cmake_minimum_required(VERSION 3.0)
project(pcl_cloud_processing)

# C++17
set(CMAKE_CXX_STANDARD 17)

# third party
set(THIRD_PARTY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../third_party)

# PCL
find_package(PCL REQUIRED)
if (PCL_FOUND)
    message(STATUS "Found PCL library: " ${PCL_INCLUDE_DIRS})
    include_directories(${PCL_INCLUDE_DIRS})
endif (PCL_FOUND)

# Include
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${THIRD_PARTY_DIR}/spdlog/include)

# Link
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Build
add_executable(downsampling downsampling.cpp)
target_link_libraries(downsampling ${PCL_LIBRARIES})




