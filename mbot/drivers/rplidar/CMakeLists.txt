cmake_minimum_required(VERSION 3.1 FATAL_ERROR)
project(rplidar_driver)

set(CMAKE_BUILD_TYPE RelWithDebInfo)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")

find_package(Threads REQUIRED)
find_package(lcm REQUIRED)

find_library(common_utils REQUIRED)
link_directories(lib/)
add_executable(rplidar_driver src/rplidar_driver.cpp)

target_link_libraries(rplidar_driver
    ${CMAKE_THREAD_LIBS_INIT}
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/librplidar_sdk.a
    lcm
    common_utils
    mbot_lcm_msgs-cpp
    pthread
)

target_include_directories(rplidar_driver PRIVATE include)
