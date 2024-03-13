cmake_minimum_required(VERSION 3.0)
project (lie_group_example)

set(SOPHUS_PATH ${CMAKE_SOURCE_DIR}/../third_party/Sophus)

find_package(Eigen3 REQUIRED)
find_package(fmt REQUIRED)

add_executable(lie_group_example lie_group_example.cpp)
target_include_directories(lie_group_example PRIVATE ${SOPHUS_PATH} ${EIGEN3_INCLUDE_DIR})
target_link_libraries(lie_group_example fmt::fmt)


