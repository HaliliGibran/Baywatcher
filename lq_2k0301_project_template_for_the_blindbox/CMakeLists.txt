cmake_minimum_required(VERSION 3.16)

set(CROSS_COMPILER_DIR
    /opt/loongson-gnu-toolchain-8.3-x86_64-loongarch64-linux-gnu-rc1.3-1)

if(NOT DEFINED CMAKE_SYSTEM_NAME)
    set(CMAKE_SYSTEM_NAME Linux)
endif()
if(NOT DEFINED CMAKE_SYSTEM_PROCESSOR)
    set(CMAKE_SYSTEM_PROCESSOR loongarch64)
endif()
if(NOT DEFINED CMAKE_C_COMPILER)
    set(CMAKE_C_COMPILER
        ${CROSS_COMPILER_DIR}/bin/loongarch64-linux-gnu-gcc)
endif()
if(NOT DEFINED CMAKE_CXX_COMPILER)
    set(CMAKE_CXX_COMPILER
        ${CROSS_COMPILER_DIR}/bin/loongarch64-linux-gnu-g++)
endif()

project(BayWatcher_Blindbox LANGUAGES C CXX)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(main
    Code/main.cpp
    Code/User/src/Buzzer.cc
    LQ_Libraries_old/Driver/LQ_HW_GPIO.cpp
    LQ_Libraries_old/Driver/LQ_MAP_ADDR.cpp
)

target_include_directories(main PRIVATE
    ${PROJECT_SOURCE_DIR}/Code/User/inc
    ${PROJECT_SOURCE_DIR}/LQ_Libraries_old/Driver/inc
)

target_compile_options(main PRIVATE -O3)
target_link_libraries(main PRIVATE pthread)
