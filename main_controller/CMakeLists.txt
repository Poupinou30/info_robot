cmake_minimum_required(VERSION 3.16)
project(main_controller C)

set(CMAKE_C_STANDARD 99)

include_directories(${CMAKE_SOURCE_DIR}/src)

find_package(pigpio REQUIRED)
#add_compile_options(-g)


add_executable(main_controller src/main.c
        src/variables.c
        src/fieldpath.c
        src/headers.h
        src/spi_com.c
        src/utils.c
        src/controller.c
        src/kalman.c
        src/feedbackOdometry.c
        src/forkControl.c
        src/forkStrategy.c
        src/mainStrategy.c
        src/initializations.c
        src/UART.c
        src/lidarManagement.c
        src/nextion.c
        )

target_include_directories(main_controller PRIVATE ${pigpio_INCLUDE_DIR})
target_link_libraries(main_controller PRIVATE ${pigpio_LIBRARY} m pthread)
