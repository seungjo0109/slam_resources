cmake_minimum_required(VERSION 3.0)
project(eigen_example)

find_package(Eigen3 REQUIRED)

add_executable(${PROJECT_NAME} eigen_example.cpp)
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)
