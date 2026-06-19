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

set(PROJECT_MODEL_DIR ${PROJECT_SOURCE_DIR}/model)
set(OUTPUT_MODEL_DIR ${PROJECT_BINARY_DIR}/model)
set(PROJECT_MODEL_GRAYRED32_DIR ${PROJECT_SOURCE_DIR}/model_mlp_wider_grayred_taskroi320_realcal_synsel_ls005_v1)
set(OUTPUT_MODEL_GRAYRED32_DIR ${PROJECT_BINARY_DIR}/model_mlp_wider_grayred_taskroi320_realcal_synsel_ls005_v1)
set(PROJECT_MODEL_GRAY32_SUBCLASS_DIR ${PROJECT_SOURCE_DIR}/model_subclass320_mlp_gray_256_rank1)
set(OUTPUT_MODEL_GRAY32_SUBCLASS_DIR ${PROJECT_BINARY_DIR}/model_subclass320_mlp_gray_256_rank1)
set(PROJECT_MODEL_RGB32_SUBCLASS_DIR ${PROJECT_SOURCE_DIR}/model_subclass320_mlp_rgb_256_128_rank3)
set(OUTPUT_MODEL_RGB32_SUBCLASS_DIR ${PROJECT_BINARY_DIR}/model_subclass320_mlp_rgb_256_128_rank3)
set(LQ_LOCAL_OPENCV_ROOT ${PROJECT_SOURCE_DIR}/cross_lib/opencv)
set(LQ_LOCAL_NEWLIB_DIR ${PROJECT_SOURCE_DIR}/libraries)
set(LQ_CAMERA_EX_LIB "")
set(LQ_CAMERA_EX_COMPAT_SRC ${PROJECT_SOURCE_DIR}/Code/User/src/lq_camera_ex_compat.cc)
set(LQ_CAMERA_EX_LIB_CANDIDATES
    ${LQ_LOCAL_NEWLIB_DIR}/drv/src/liblq_camera_ex.a
    ${PROJECT_SOURCE_DIR}/LQ_Libraries_new/drv/src/liblq_camera_ex.a
    ${PROJECT_SOURCE_DIR}/../lq_2k0301_project_template_for_running/LQ_Libraries_new/drv/src/liblq_camera_ex.a
)

foreach(_camera_lib_candidate IN LISTS LQ_CAMERA_EX_LIB_CANDIDATES)
    if (EXISTS "${_camera_lib_candidate}")
        set(LQ_CAMERA_EX_LIB "${_camera_lib_candidate}")
        break()
    endif()
endforeach()

set(LQ_USE_CAMERA_EX_COMPAT OFF)
set(LQ_CAMERA_EX_LIB_HAS_ADVANCED_CONTROLS OFF)
set(LQ_CAMERA_EX_REQUIRED_SYMBOLS
    "lq_camera_ex::set_exposure_auto()"
    "lq_camera_ex::set_gain_manual(double)"
    "lq_camera_ex::set_white_balance_manual(double, double)"
    "lq_camera_ex::set_white_balance_auto()"
    "lq_camera_ex::get_exposure_value() const"
    "lq_camera_ex::get_gain_value() const"
    "lq_camera_ex::get_white_balance_blue_u_value() const"
    "lq_camera_ex::get_white_balance_red_v_value() const"
)

if (EXISTS "${LQ_CAMERA_EX_LIB}")
    if (NOT CMAKE_NM)
        find_program(CMAKE_NM NAMES
            loongarch64-linux-gnu-nm
            llvm-nm
            nm)
    endif()

    if (CMAKE_NM)
        execute_process(
            COMMAND ${CMAKE_NM} -C "${LQ_CAMERA_EX_LIB}"
            OUTPUT_VARIABLE LQ_CAMERA_EX_NM_OUTPUT
            RESULT_VARIABLE LQ_CAMERA_EX_NM_RESULT
            ERROR_QUIET)

        if (LQ_CAMERA_EX_NM_RESULT EQUAL 0)
            set(LQ_CAMERA_EX_LIB_HAS_ADVANCED_CONTROLS ON)
            foreach(_camera_required_symbol IN LISTS LQ_CAMERA_EX_REQUIRED_SYMBOLS)
                string(FIND "${LQ_CAMERA_EX_NM_OUTPUT}" "${_camera_required_symbol}" _camera_symbol_pos)
                if (_camera_symbol_pos EQUAL -1)
                    set(LQ_CAMERA_EX_LIB_HAS_ADVANCED_CONTROLS OFF)
                    set(LQ_CAMERA_EX_FIRST_MISSING_SYMBOL "${_camera_required_symbol}")
                    break()
                endif()
            endforeach()
        endif()
    endif()
endif()

if (EXISTS "${LQ_CAMERA_EX_LIB}" AND NOT LQ_CAMERA_EX_LIB_HAS_ADVANCED_CONTROLS)
    if (EXISTS "${LQ_CAMERA_EX_COMPAT_SRC}")
        set(LQ_USE_CAMERA_EX_COMPAT ON)
        message(WARNING
            "检测到现有 lq_camera_ex 静态库缺少高级相机控制接口，自动改用源码兼容实现：\n"
            "  lib: ${LQ_CAMERA_EX_LIB}\n"
            "  missing: ${LQ_CAMERA_EX_FIRST_MISSING_SYMBOL}\n"
            "  compat: ${LQ_CAMERA_EX_COMPAT_SRC}")
    else()
        message(FATAL_ERROR
            "现有 lq_camera_ex 静态库缺少高级相机控制接口，且兼容源码不存在。\n"
            "  lib: ${LQ_CAMERA_EX_LIB}\n"
            "  missing: ${LQ_CAMERA_EX_FIRST_MISSING_SYMBOL}\n"
            "请补齐 ${LQ_CAMERA_EX_COMPAT_SRC}，或替换为包含这些接口的新版本 liblq_camera_ex.a。")
    endif()
endif()

if (NOT EXISTS "${LQ_CAMERA_EX_LIB}")
    if (EXISTS "${LQ_CAMERA_EX_COMPAT_SRC}")
        set(LQ_USE_CAMERA_EX_COMPAT ON)
        message(WARNING
            "未找到 lq_camera_ex 静态库，改用源码兼容实现：\n"
            "  ${LQ_CAMERA_EX_COMPAT_SRC}\n"
            "已检查路径：\n"
            "  ${LQ_CAMERA_EX_LIB_CANDIDATES}")
    else()
        message(FATAL_ERROR
            "未找到 lq_camera_ex 静态库，且兼容源码不存在。\n"
            "已检查以下路径：\n"
            "  ${LQ_CAMERA_EX_LIB_CANDIDATES}\n"
            "请确认识别板或运行板目录中至少有一份 liblq_camera_ex.a 可用，"
            "或者补齐 ${LQ_CAMERA_EX_COMPAT_SRC}。")
    endif()
endif()

# 识别板当前只保留“彩色采集 + 图传 + 识别链 + UART”这一圈真实依赖。
# 旧单板控制/菜单/IMU/ncnn 相关路径不再进入默认构建。
include_directories(${PROJECT_SOURCE_DIR}/LQ_Libraries_old/Driver/inc)
include_directories(${PROJECT_SOURCE_DIR}/LQ_Libraries_new/common/inc)
include_directories(${PROJECT_SOURCE_DIR}/LQ_Libraries_new/drv/inc)
include_directories(${LQ_LOCAL_NEWLIB_DIR}/common/inc)
include_directories(${LQ_LOCAL_NEWLIB_DIR}/drv/inc)

include_directories(${PROJECT_SOURCE_DIR}/Code/User/inc)
include_directories(${PROJECT_SOURCE_DIR}/Code/User)
include_directories(${PROJECT_SOURCE_DIR}/Code)

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
            EXISTS "${CANDIDATE_ROOT}/lib/libopencv_dnn.so" AND
            (NOT LQ_USE_CAMERA_EX_COMPAT OR EXISTS "${CANDIDATE_ROOT}/lib/libopencv_videoio.so"))
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
set(LQ_OPENCV_COMPONENTS core imgcodecs imgproc dnn)
if (LQ_USE_CAMERA_EX_COMPAT)
    list(APPEND LQ_OPENCV_COMPONENTS videoio)
endif()

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
            "  lib/libopencv_dnn.so\n"
            "  lib/libopencv_videoio.so （当启用 lq_camera_ex 兼容实现时）\n"
            "或者包含 OpenCVConfig.cmake / opencv-config.cmake。")
    endif()
endif()

set(PLATFORM_SRC
    ${PROJECT_SOURCE_DIR}/LQ_Libraries_old/Driver/WW_Transmission.cpp
    ${PROJECT_SOURCE_DIR}/LQ_Libraries_new/drv/src/lq_uart.cpp
)

set(RECOGNITION_BOARD_SRC
    ${PROJECT_SOURCE_DIR}/Code/main.cpp
    ${PROJECT_SOURCE_DIR}/Code/User/src/Communication.cc
    ${PROJECT_SOURCE_DIR}/Code/User/src/latest_frame_grabber.cc
    ${PROJECT_SOURCE_DIR}/Code/User/src/recognition_runtime.cc
    ${PROJECT_SOURCE_DIR}/Code/User/src/recognition_chain.cc
    ${PROJECT_SOURCE_DIR}/Code/User/src/roi_capture_runtime.cc
    ${PROJECT_SOURCE_DIR}/Code/User/src/roi_runtime_geometry.cc
    ${PROJECT_SOURCE_DIR}/Code/User/src/stream_chain.cc
    ${PROJECT_SOURCE_DIR}/Code/User/src/transform_table.cc
)

if (LQ_USE_CAMERA_EX_COMPAT)
    list(APPEND RECOGNITION_BOARD_SRC ${LQ_CAMERA_EX_COMPAT_SRC})
endif()

if (LQ_OPENCV_OK AND LQ_OPENCV_MODE STREQUAL "MANUAL")
    link_directories(${LQ_OPENCV_LIB_DIR})
endif()

# 由源文件生成一个可执行程序
add_executable(${BIN} ${PLATFORM_SRC} ${RECOGNITION_BOARD_SRC})

file(MAKE_DIRECTORY ${OUTPUT_MODEL_DIR})
configure_file(${PROJECT_MODEL_DIR}/cls.onnx ${OUTPUT_MODEL_DIR}/cls.onnx COPYONLY)
configure_file(${PROJECT_MODEL_DIR}/class_names.json ${OUTPUT_MODEL_DIR}/class_names.json COPYONLY)
configure_file(${PROJECT_MODEL_DIR}/deploy_calibration.json ${OUTPUT_MODEL_DIR}/deploy_calibration.json COPYONLY)

if (EXISTS "${PROJECT_MODEL_GRAYRED32_DIR}/cls.onnx" AND
    EXISTS "${PROJECT_MODEL_GRAYRED32_DIR}/class_names.json" AND
    EXISTS "${PROJECT_MODEL_GRAYRED32_DIR}/deploy_calibration.json")
    file(MAKE_DIRECTORY ${OUTPUT_MODEL_GRAYRED32_DIR})
    configure_file(${PROJECT_MODEL_GRAYRED32_DIR}/cls.onnx ${OUTPUT_MODEL_GRAYRED32_DIR}/cls.onnx COPYONLY)
    configure_file(${PROJECT_MODEL_GRAYRED32_DIR}/class_names.json ${OUTPUT_MODEL_GRAYRED32_DIR}/class_names.json COPYONLY)
    configure_file(${PROJECT_MODEL_GRAYRED32_DIR}/deploy_calibration.json ${OUTPUT_MODEL_GRAYRED32_DIR}/deploy_calibration.json COPYONLY)
endif()

if (EXISTS "${PROJECT_MODEL_GRAY32_SUBCLASS_DIR}/cls.onnx" AND
    EXISTS "${PROJECT_MODEL_GRAY32_SUBCLASS_DIR}/class_names.json")
    file(MAKE_DIRECTORY ${OUTPUT_MODEL_GRAY32_SUBCLASS_DIR})
    configure_file(${PROJECT_MODEL_GRAY32_SUBCLASS_DIR}/cls.onnx ${OUTPUT_MODEL_GRAY32_SUBCLASS_DIR}/cls.onnx COPYONLY)
    configure_file(${PROJECT_MODEL_GRAY32_SUBCLASS_DIR}/class_names.json ${OUTPUT_MODEL_GRAY32_SUBCLASS_DIR}/class_names.json COPYONLY)
    if (EXISTS "${PROJECT_MODEL_GRAY32_SUBCLASS_DIR}/deploy_calibration.json")
        configure_file(${PROJECT_MODEL_GRAY32_SUBCLASS_DIR}/deploy_calibration.json ${OUTPUT_MODEL_GRAY32_SUBCLASS_DIR}/deploy_calibration.json COPYONLY)
    endif()
endif()

if (EXISTS "${PROJECT_MODEL_RGB32_SUBCLASS_DIR}/cls.onnx" AND
    EXISTS "${PROJECT_MODEL_RGB32_SUBCLASS_DIR}/class_names.json")
    file(MAKE_DIRECTORY ${OUTPUT_MODEL_RGB32_SUBCLASS_DIR})
    configure_file(${PROJECT_MODEL_RGB32_SUBCLASS_DIR}/cls.onnx ${OUTPUT_MODEL_RGB32_SUBCLASS_DIR}/cls.onnx COPYONLY)
    configure_file(${PROJECT_MODEL_RGB32_SUBCLASS_DIR}/class_names.json ${OUTPUT_MODEL_RGB32_SUBCLASS_DIR}/class_names.json COPYONLY)
    if (EXISTS "${PROJECT_MODEL_RGB32_SUBCLASS_DIR}/deploy_calibration.json")
        configure_file(${PROJECT_MODEL_RGB32_SUBCLASS_DIR}/deploy_calibration.json ${OUTPUT_MODEL_RGB32_SUBCLASS_DIR}/deploy_calibration.json COPYONLY)
    endif()
endif()

#Opencv_lib
if (LQ_OPENCV_OK AND LQ_OPENCV_MODE STREQUAL "MANUAL")
    set(OpenCV_LIBS opencv_core opencv_imgcodecs opencv_imgproc opencv_dnn)
    if (LQ_USE_CAMERA_EX_COMPAT)
        list(APPEND OpenCV_LIBS opencv_videoio)
    endif()
else()
    set(OpenCV_LIBS ${OpenCV_LIBS})
endif()

#System_lib
set(SYSTEM_LIBS pthread gomp)
if (NOT LQ_USE_CAMERA_EX_COMPAT)
    list(APPEND SYSTEM_LIBS ${LQ_CAMERA_EX_LIB})
endif()

set(ALL_LIBS ${OpenCV_LIBS} ${SYSTEM_LIBS})

target_link_libraries(${BIN} ${ALL_LIBS})
