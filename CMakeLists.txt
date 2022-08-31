cmake_minimum_required(VERSION 3.5)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(undistortX)

find_package(Ceres REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)

set(SOURCES
    sources/geometry.h
    sources/types.h
    sources/undistortion.h
    sources/undistortion.cpp
)

set(PROJECT_SOURCES
    sources/main.cpp
)

set(TEST_SOURCES
    tests/test.cpp
)

add_executable(
    ${PROJECT_NAME} 
    ${SOURCES}
    ${PROJECT_SOURCES}
)

target_link_libraries(
    ${PROJECT_NAME} 
    Ceres::ceres
    Eigen3::Eigen
    ${OpenCV_LIBS}
)

target_include_directories(
    ${PROJECT_NAME}
    PRIVATE
    sources
    ${OpenCV_INCLUDE_DIRS}
)

add_executable(
    ${PROJECT_NAME}_TEST 
    ${SOURCES}
    ${TEST_SOURCES}
)

target_link_libraries(
    ${PROJECT_NAME}_TEST 
    Ceres::ceres
    Eigen3::Eigen
    ${OpenCV_LIBS}
)

target_include_directories(
    ${PROJECT_NAME}_TEST 
    PRIVATE
    sources
    ${OpenCV_INCLUDE_DIRS}
)
