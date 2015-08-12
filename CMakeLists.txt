cmake_minimum_required (VERSION 2.8)

project(vsa_knee)

add_executable (program
    main.c declare.c timer_thread.c encoder_thread.c
magnet_thread.c energy_thread.c utilities.c)

target_link_libraries(program bcm2835 pthread rt m)
