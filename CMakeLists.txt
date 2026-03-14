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

# 指定所用的头文件路径
include_directories(${PROJECT_SOURCE_DIR}/LQ_Libraries_old/Driver/inc)
include_directories(${PROJECT_SOURCE_DIR}/LQ_Libraries_old/APP/Display)
include_directories(${PROJECT_SOURCE_DIR}/LQ_Libraries_old/APP/IMU)
include_directories(${PROJECT_SOURCE_DIR}/LQ_Libraries_old/APP/inc)

include_directories(${PROJECT_SOURCE_DIR}/LQ_Libraries_new/common/inc)
include_directories(${PROJECT_SOURCE_DIR}/LQ_Libraries_new/app/display)
include_directories(${PROJECT_SOURCE_DIR}/LQ_Libraries_new/app/gyro)
include_directories(${PROJECT_SOURCE_DIR}/LQ_Libraries_new/app)
include_directories(${PROJECT_SOURCE_DIR}/LQ_Libraries_new/drv/inc)

include_directories(${PROJECT_SOURCE_DIR}/Code/User/image/element)
include_directories(${PROJECT_SOURCE_DIR}/Code/User/image)
include_directories(${PROJECT_SOURCE_DIR}/Code/User/inc)
include_directories(${PROJECT_SOURCE_DIR}/Code/User)
include_directories(${PROJECT_SOURCE_DIR}/Code)

# 指定OpenCV所用头文件路径 --> (需根据自己实际路径进行修改)
include_directories(${PROJECT_SOURCE_DIR}/cross_lib/opencv/include/opencv4)

# 指定ncnn所用头文件路径
include_directories(${PROJECT_SOURCE_DIR}/cross_lib/ncnn/include)

file(GLOB_RECURSE MYSRC
    ${PROJECT_SOURCE_DIR}/LQ_Libraries_old/*.cc
    ${PROJECT_SOURCE_DIR}/LQ_Libraries_old/*.cpp  
    ${PROJECT_SOURCE_DIR}/LQ_Libraries_old/*.c

    ${PROJECT_SOURCE_DIR}/LQ_Libraries_new/*.cc
    ${PROJECT_SOURCE_DIR}/LQ_Libraries_new/*.cpp  
    ${PROJECT_SOURCE_DIR}/LQ_Libraries_new/*.c

    ${PROJECT_SOURCE_DIR}/Code/*.cc
    ${PROJECT_SOURCE_DIR}/Code/*.cpp  
    ${PROJECT_SOURCE_DIR}/Code/*.c
)

link_directories(${PROJECT_SOURCE_DIR}/cross_lib/opencv/lib)

link_directories(${PROJECT_SOURCE_DIR}/cross_lib/ncnn/lib)


# 由源文件生成一个可执行程序
add_executable(${BIN} ${MYSRC})

file(MAKE_DIRECTORY ${OUTPUT_MODEL_DIR})
configure_file(${PROJECT_MODEL_DIR}/cls.onnx ${OUTPUT_MODEL_DIR}/cls.onnx COPYONLY)
configure_file(${PROJECT_MODEL_DIR}/class_names.json ${OUTPUT_MODEL_DIR}/class_names.json COPYONLY)

#Opencv_lib
set(OpenCV_LIBS opencv_core opencv_imgcodecs opencv_imgproc opencv_videoio opencv_dnn)

#System_lib
set(SYSTEM_LIBS pthread gomp)

set(ALL_LIBS ${OpenCV_LIBS} ncnn ${SYSTEM_LIBS})

target_link_libraries(${BIN} ${ALL_LIBS})



