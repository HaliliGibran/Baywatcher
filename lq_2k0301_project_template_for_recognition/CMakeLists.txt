cmake_minimum_required(VERSION 3.16)    # 限制最低CMake版本

# 项目名称
project(LQ_2K0301_Project)

# BIN后面可以替换成自己想要生成文件的名字
set(BIN main)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 设置交叉编译工具路径 --> (需根据自己实际路径进行修改)
set(CROSS_COMPILER_DIR /opt/loongson-gnu-toolchain-8.3-x86_64-loongarch64-linux-gnu-rc1.3-1)
set(CMAKE_C_COMPILER ${CROSS_COMPILER_DIR}/bin/loongarch64-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER ${CROSS_COMPILER_DIR}/bin/loongarch64-linux-gnu-g++)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

set(PROJECT_MODEL_DIR ${PROJECT_SOURCE_DIR}/model)
set(OUTPUT_MODEL_DIR ${PROJECT_BINARY_DIR}/model)

# 识别板当前只保留“彩色采集 + 图传 + 识别链 + UART”这一圈真实依赖。
# 旧单板控制/菜单/IMU/ncnn 相关路径不再进入默认构建。
include_directories(${PROJECT_SOURCE_DIR}/LQ_Libraries_old/Driver/inc)
include_directories(${PROJECT_SOURCE_DIR}/LQ_Libraries_new/common/inc)
include_directories(${PROJECT_SOURCE_DIR}/LQ_Libraries_new/drv/inc)

include_directories(${PROJECT_SOURCE_DIR}/Code/User/inc)
include_directories(${PROJECT_SOURCE_DIR}/Code/User)
include_directories(${PROJECT_SOURCE_DIR}/Code)

# 指定OpenCV所用头文件路径 --> (需根据自己实际路径进行修改)
include_directories(${PROJECT_SOURCE_DIR}/cross_lib/opencv/include/opencv4)

set(PLATFORM_SRC
    ${PROJECT_SOURCE_DIR}/LQ_Libraries_old/Driver/WW_CAMERA.cpp
    ${PROJECT_SOURCE_DIR}/LQ_Libraries_old/Driver/WW_Transmission.cpp
    ${PROJECT_SOURCE_DIR}/LQ_Libraries_new/drv/src/lq_uart.cpp
)

set(RECOGNITION_BOARD_SRC
    ${PROJECT_SOURCE_DIR}/Code/main.cpp
    ${PROJECT_SOURCE_DIR}/Code/User/src/Communication.cc
    ${PROJECT_SOURCE_DIR}/Code/User/src/recognition_runtime.cc
    ${PROJECT_SOURCE_DIR}/Code/User/src/recognition_chain.cc
    ${PROJECT_SOURCE_DIR}/Code/User/src/stream_chain.cc
    ${PROJECT_SOURCE_DIR}/Code/User/src/transform_table.cc
)

link_directories(${PROJECT_SOURCE_DIR}/cross_lib/opencv/lib)

# 由源文件生成一个可执行程序
add_executable(${BIN} ${PLATFORM_SRC} ${RECOGNITION_BOARD_SRC})

file(MAKE_DIRECTORY ${OUTPUT_MODEL_DIR})
configure_file(${PROJECT_MODEL_DIR}/cls.onnx ${OUTPUT_MODEL_DIR}/cls.onnx COPYONLY)
configure_file(${PROJECT_MODEL_DIR}/class_names.json ${OUTPUT_MODEL_DIR}/class_names.json COPYONLY)

#Opencv_lib
set(OpenCV_LIBS opencv_core opencv_imgcodecs opencv_imgproc opencv_dnn)

#System_lib
set(SYSTEM_LIBS pthread gomp)

set(ALL_LIBS ${OpenCV_LIBS} ${SYSTEM_LIBS})

target_link_libraries(${BIN} ${ALL_LIBS})
