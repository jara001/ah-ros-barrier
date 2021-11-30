cmake_minimum_required(VERSION 3.1.0)

project(optic_barrier_sw)

# If you want to use different compiler than arm-linux-gnueabihf-gcc, run cmake as:
# cmake -DHARDCODE_C_COMPILER=NO -DCMAKE_C_COMPILER=...
set(HARDCODE_C_COMPILER YES CACHE BOOL "Override user's (or default) value of CMAKE_C_COMPILER with 'arm-linux-gnueabihf-gcc'")

if(HARDCODE_C_COMPILER)
  set(CMAKE_C_COMPILER arm-linux-gnueabihf-gcc)
  #set(CMAKE_C_COMPILER gcc)
endif()
message(STATUS "CMAKE_C_COMPILER: ${CMAKE_C_COMPILER}")

set (CMAKE_C_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -fno-omit-frame-pointer -fsanitize=address -static-libasan")
set (CMAKE_LINKER_FLAGS_DEBUG "${CMAKE_LINKER_FLAGS_DEBUG} -fno-omit-frame-pointer -fsanitize=address -static-libasan")

link_directories(${CMAKE_SOURCE_DIR} wiringPi/)

INCLUDE_DIRECTORIES("wiringPi" "OLED/Fonts" "OLED/Driver")

set(SRCS main.c OLED/Driver/OLED_Config.c OLED/Driver/OLED_Driver.c OLED/Driver/OLED_GUI.c OLED/Fonts/font8.c OLED/Fonts/font12.c OLED/Fonts/font16.c OLED/Fonts/font20.c OLED/Fonts/font24.c) 

add_executable(${PROJECT_NAME} ${HDRS} ${SRCS} )

target_link_libraries(${PROJECT_NAME} -lwiringPi -lpthread)

if(DEFINED ENV{IN_NIX_SHELL})
  # When compiled by Nix, we need to change interpreter in order to
  # run the resulting binary on Raspbian system.
  add_custom_target(patchelf ALL
    COMMAND patchelf --set-interpreter /lib/ld-linux-armhf.so.3 ${PROJECT_NAME}
    DEPENDS ${PROJECT_NAME}
    )
endif()
