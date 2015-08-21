cmake_minimum_required (VERSION 2.8.9)

project(pd_motor_control)

if("${CMAKE_C_COMPILER_ID}" STREQUAL "GNU")
    # require at least gcc 4.7
    if (CMAKE_C_COMPILER_VERSION VERSION_LESS 4.7)
        message(FATAL_ERROR "GCC version must be at least 4.7!")
    endif()
else()
    message(FATAL_ERROR "You are using an unsupported compiler! Compilation has only been tested with GCC.")
endif()

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=gnu99 -O0 -ggdb")

aux_source_directory(. SRC_LIST)
add_executable(${PROJECT_NAME} ${SRC_LIST})
target_link_libraries(${PROJECT_NAME} bcm2835 pthread rt m)
