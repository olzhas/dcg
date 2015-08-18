cmake_minimum_required (VERSION 2.8)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=gnu99")

project(vsa_knee)

add_executable (program
main.c declare.c tasks.c utils.c)

target_link_libraries(program bcm2835 pthread rt m)
