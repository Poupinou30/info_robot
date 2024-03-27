cmake_minimum_required(VERSION 3.16)
project(main_controller C)

set(CMAKE_C_STANDARD 99)

include_directories(.)
find_package(pigpio REQUIRED)

add_executable(main_controller main.c
        variables.c
        fieldpath.c
        headers.h
        spi_com.c
        utils.c
        controller.c
        kalman.c
        feedbackOdometry.c
        forkControl.c
        strategy.c
        )

target_include_directories(main_controller PRIVATE ${pigpio_INCLUDE_DIR})
target_link_libraries(main_controller PRIVATE ${pigpio_LIBRARY} m pthread)
