cmake_minimum_required(VERSION 3.0.2)
project(cartesianPlanner)

add_definitions(-std=c++11 -w)

include_directories(
    ./include
)

aux_source_directory(./src MYSRC)

add_executable(${PROJECT_NAME}_demo ${MYSRC})

target_link_libraries(${PROJECT_NAME}_demo -lnlopt /usr/local/lib/libcasadi.so)
