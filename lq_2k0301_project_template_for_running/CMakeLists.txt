cmake_minimum_required(VERSION 3.16)    # 限制最低CMake版本

# 设置交叉编译工具路径 --> (需根据自己实际路径进行修改)
set(CROSS_COMPILER_DIR /opt/loongson-gnu-toolchain-8.3-x86_64-loongarch64-linux-gnu-rc1.3-1)
if (NOT DEFINED CMAKE_SYSTEM_NAME)
    set(CMAKE_SYSTEM_NAME Linux)
endif()
if (NOT DEFINED CMAKE_SYSTEM_PROCESSOR)
    set(CMAKE_SYSTEM_PROCESSOR loongarch64)
endif()
if (NOT DEFINED CMAKE_C_COMPILER)
    set(CMAKE_C_COMPILER ${CROSS_COMPILER_DIR}/bin/loongarch64-linux-gnu-gcc)
endif()
if (NOT DEFINED CMAKE_CXX_COMPILER)
    set(CMAKE_CXX_COMPILER ${CROSS_COMPILER_DIR}/bin/loongarch64-linux-gnu-g++)
endif()

# 项目名称
project(LQ_2K0301_Project)

# BIN后面可以替换成自己想要生成文件的名字
set(BIN main)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

set(LQ_LOCAL_OPENCV_ROOT ${PROJECT_SOURCE_DIR}/cross_lib/opencv)
set(LQ_LOCAL_NCNN_ROOT ${PROJECT_SOURCE_DIR}/cross_lib/ncnn)
option(BW_RUNNING_ENABLE_LEGACY_NCNN "构建旧单板本地 ncnn 识别链" OFF)

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
include_directories(${PROJECT_SOURCE_DIR}/Code/User/image/)
include_directories(${PROJECT_SOURCE_DIR}/Code/User/inc)
include_directories(${PROJECT_SOURCE_DIR}/Code/User)
include_directories(${PROJECT_SOURCE_DIR}/Code)

if (BW_RUNNING_ENABLE_LEGACY_NCNN)
    include_directories(${LQ_LOCAL_NCNN_ROOT}/include)
endif()

set(LQ_OPENCV_ROOT "" CACHE PATH "完整 OpenCV 根目录，结构需包含 include/opencv4 与 lib，或包含 OpenCVConfig.cmake")

function(lq_try_use_opencv_root CANDIDATE_ROOT OUT_OK OUT_MODE OUT_INCLUDE OUT_LIBDIR OUT_CONFIGDIR)
    set(_ok FALSE)
    set(_mode "")
    set(_include "")
    set(_libdir "")
    set(_configdir "")

    if (EXISTS "${CANDIDATE_ROOT}/lib/cmake/opencv4/OpenCVConfig.cmake")
        set(_ok TRUE)
        set(_mode "CONFIG")
        set(_configdir "${CANDIDATE_ROOT}/lib/cmake/opencv4")
    elseif (EXISTS "${CANDIDATE_ROOT}/include/opencv4/opencv2/core/cvdef.h" AND
            EXISTS "${CANDIDATE_ROOT}/include/opencv4/opencv2/dnn/dnn.hpp" AND
            EXISTS "${CANDIDATE_ROOT}/lib/libopencv_core.so" AND
            EXISTS "${CANDIDATE_ROOT}/lib/libopencv_imgproc.so" AND
            EXISTS "${CANDIDATE_ROOT}/lib/libopencv_imgcodecs.so" AND
            EXISTS "${CANDIDATE_ROOT}/lib/libopencv_videoio.so" AND
            EXISTS "${CANDIDATE_ROOT}/lib/libopencv_dnn.so")
        set(_ok TRUE)
        set(_mode "MANUAL")
        set(_include "${CANDIDATE_ROOT}/include/opencv4")
        set(_libdir "${CANDIDATE_ROOT}/lib")
    endif()

    set(${OUT_OK} ${_ok} PARENT_SCOPE)
    set(${OUT_MODE} "${_mode}" PARENT_SCOPE)
    set(${OUT_INCLUDE} "${_include}" PARENT_SCOPE)
    set(${OUT_LIBDIR} "${_libdir}" PARENT_SCOPE)
    set(${OUT_CONFIGDIR} "${_configdir}" PARENT_SCOPE)
endfunction()

set(LQ_OPENCV_OK FALSE)
set(LQ_OPENCV_MODE "")
set(LQ_OPENCV_INCLUDE_DIR "")
set(LQ_OPENCV_LIB_DIR "")
set(LQ_OPENCV_CONFIG_DIR "")
set(LQ_OPENCV_COMPONENTS core imgcodecs imgproc videoio dnn)

if (LQ_OPENCV_ROOT)
    lq_try_use_opencv_root("${LQ_OPENCV_ROOT}" LQ_OPENCV_OK LQ_OPENCV_MODE LQ_OPENCV_INCLUDE_DIR LQ_OPENCV_LIB_DIR LQ_OPENCV_CONFIG_DIR)
endif()

if (NOT LQ_OPENCV_OK)
    lq_try_use_opencv_root("${LQ_LOCAL_OPENCV_ROOT}" LQ_OPENCV_OK LQ_OPENCV_MODE LQ_OPENCV_INCLUDE_DIR LQ_OPENCV_LIB_DIR LQ_OPENCV_CONFIG_DIR)
endif()

if (LQ_OPENCV_OK AND LQ_OPENCV_MODE STREQUAL "CONFIG")
    set(OpenCV_DIR "${LQ_OPENCV_CONFIG_DIR}" CACHE PATH "Bundled OpenCV config directory" FORCE)
    find_package(OpenCV REQUIRED CONFIG COMPONENTS ${LQ_OPENCV_COMPONENTS})
    include_directories(${OpenCV_INCLUDE_DIRS})
elseif (LQ_OPENCV_OK AND LQ_OPENCV_MODE STREQUAL "MANUAL")
    include_directories(${LQ_OPENCV_INCLUDE_DIR})
else()
    find_package(OpenCV QUIET CONFIG COMPONENTS ${LQ_OPENCV_COMPONENTS})
    if (OpenCV_FOUND)
        include_directories(${OpenCV_INCLUDE_DIRS})
    else()
        message(FATAL_ERROR
            "未找到可用的 OpenCV。\n"
            "请执行以下任一操作后重新配置：\n"
            "  1) 把完整 OpenCV 包放到 ${LQ_LOCAL_OPENCV_ROOT}\n"
            "  2) 显式传入 -DLQ_OPENCV_ROOT=/path/to/opencv\n"
            "完整包至少应包含：\n"
            "  include/opencv4/opencv2/core/cvdef.h\n"
            "  include/opencv4/opencv2/dnn/dnn.hpp\n"
            "  lib/libopencv_core.so\n"
            "  lib/libopencv_imgproc.so\n"
            "  lib/libopencv_imgcodecs.so\n"
            "  lib/libopencv_videoio.so\n"
            "  lib/libopencv_dnn.so\n"
            "或者包含 OpenCVConfig.cmake / opencv-config.cmake。")
    endif()
endif()

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

if (NOT BW_RUNNING_ENABLE_LEGACY_NCNN)
    list(FILTER MYSRC EXCLUDE REGEX ".*/lq_ncnn\\.cpp$")
endif()

add_executable(${BIN} ${MYSRC})

if (LQ_OPENCV_OK AND LQ_OPENCV_MODE STREQUAL "MANUAL")
    set(OpenCV_LIBS
        ${LQ_OPENCV_LIB_DIR}/libopencv_core.so
        ${LQ_OPENCV_LIB_DIR}/libopencv_imgcodecs.so
        ${LQ_OPENCV_LIB_DIR}/libopencv_imgproc.so
        ${LQ_OPENCV_LIB_DIR}/libopencv_videoio.so
        ${LQ_OPENCV_LIB_DIR}/libopencv_dnn.so
    )
else()
    set(OpenCV_LIBS ${OpenCV_LIBS})
endif()

set(NCNN_LIBS)
if (BW_RUNNING_ENABLE_LEGACY_NCNN)
    if (EXISTS "${LQ_LOCAL_NCNN_ROOT}/lib/libncnn.so")
        list(APPEND NCNN_LIBS ${LQ_LOCAL_NCNN_ROOT}/lib/libncnn.so)
    else()
        message(FATAL_ERROR
            "BW_RUNNING_ENABLE_LEGACY_NCNN=ON，但未找到 ${LQ_LOCAL_NCNN_ROOT}/lib/libncnn.so")
    endif()
endif()

set(SYSTEM_LIBS pthread gomp)
set(ALL_LIBS ${OpenCV_LIBS} ${NCNN_LIBS} ${SYSTEM_LIBS})

target_link_libraries(${BIN} ${ALL_LIBS})
