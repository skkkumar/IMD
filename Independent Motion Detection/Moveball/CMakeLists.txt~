cmake_minimum_required(VERSION 2.6)

project(yarp-basics)

find_package(YARP REQUIRED)

include_directories(${YARP_INCLUDE_DIRS})

add_executable(move_ball move_ball.cpp)
target_link_libraries(move_ball ${YARP_LIBRARIES})


install(TARGETS move_ball RUNTIME DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/../bin)


