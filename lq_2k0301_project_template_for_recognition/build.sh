#!/bin/bash

# 该指令表示任意指令执行失败，立即终止脚本
set -e

# # 默认开启图传
# CMAKE_STREAM_OPT="-DBW_ENABLE_STREAM=ON"
#
# # 如果运行脚本时带有 "off" 参数，则关闭图传
# if [ "$1" == "off" ]; then
#     echo -e "\n[配置] 检测到参数 'off' -> 图传功能已关闭"
#     CMAKE_STREAM_OPT="-DBW_ENABLE_STREAM=OFF"
# else
#     echo -e "\n[配置] 默认编译 -> 图传功能已开启"
# fi

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
build_dir="$script_dir/output"
cache_file="$build_dir/CMakeCache.txt"
expected_cc="/opt/loongson-gnu-toolchain-8.3-x86_64-loongarch64-linux-gnu-rc1.3-1/bin/loongarch64-linux-gnu-gcc"
expected_cxx="/opt/loongson-gnu-toolchain-8.3-x86_64-loongarch64-linux-gnu-rc1.3-1/bin/loongarch64-linux-gnu-g++"

echo -e "\n=== 构建 build目录 ==="

if [ -f "$cache_file" ]; then
    cached_home="$(sed -n 's#^CMAKE_HOME_DIRECTORY:INTERNAL=##p' "$cache_file" | tr -d '\r')"
    cached_cc="$(sed -n 's#^CMAKE_C_COMPILER:FILEPATH=##p' "$cache_file" | tr -d '\r')"
    cached_cxx="$(sed -n 's#^CMAKE_CXX_COMPILER:FILEPATH=##p' "$cache_file" | tr -d '\r')"
    need_clean=0

    if [ -n "$cached_home" ] && [ "$cached_home" != "$script_dir" ]; then
        echo "[清理] 检测到旧的 CMake 缓存来自: $cached_home"
        echo "[清理] 当前源码目录为: $script_dir"
        need_clean=1
    fi

    if [ -n "$cached_cc" ] && [ "$cached_cc" != "$expected_cc" ]; then
        echo "[清理] 检测到旧的 C 编译器缓存: $cached_cc"
        echo "[清理] 当前期望 C 编译器: $expected_cc"
        need_clean=1
    fi

    if [ -n "$cached_cxx" ] && [ "$cached_cxx" != "$expected_cxx" ]; then
        echo "[清理] 检测到旧的 C++ 编译器缓存: $cached_cxx"
        echo "[清理] 当前期望 C++ 编译器: $expected_cxx"
        need_clean=1
    fi

    if [ "$need_clean" -eq 1 ]; then
        rm -rf "$build_dir"
    fi
fi

cmake -S "$script_dir" -B "$build_dir"
# cmake -S "$script_dir" -B "$build_dir" $CMAKE_STREAM_OPT

if command -v nproc >/dev/null 2>&1; then
    JOBS="$(nproc)"
elif command -v getconf >/dev/null 2>&1; then
    JOBS="$(getconf _NPROCESSORS_ONLN 2>/dev/null || echo 4)"
else
    JOBS=4
fi

pushd "$build_dir"
make -j"${JOBS}"

if [ -f main ]; then
    echo -e "\n===== 编译成功 ====="
    mkdir -p model
    cp -f ../model/cls.onnx model/cls.onnx
    cp -f ../model/class_names.json model/class_names.json
    cp -f ../model/deploy_calibration.json model/deploy_calibration.json
    if [ -d ../model_mlp_wider_grayred_taskroi320_realcal_synsel_ls005_v1 ]; then
        mkdir -p model_mlp_wider_grayred_taskroi320_realcal_synsel_ls005_v1
        cp -f ../model_mlp_wider_grayred_taskroi320_realcal_synsel_ls005_v1/cls.onnx model_mlp_wider_grayred_taskroi320_realcal_synsel_ls005_v1/cls.onnx
        cp -f ../model_mlp_wider_grayred_taskroi320_realcal_synsel_ls005_v1/class_names.json model_mlp_wider_grayred_taskroi320_realcal_synsel_ls005_v1/class_names.json
        cp -f ../model_mlp_wider_grayred_taskroi320_realcal_synsel_ls005_v1/deploy_calibration.json model_mlp_wider_grayred_taskroi320_realcal_synsel_ls005_v1/deploy_calibration.json
    fi
    #这里可以改成scp传输到我们的板卡上
    # scp main root@172.20.10.9:/home/root/workspace
    # ssh root@172.20.10.9 "mkdir -p /home/root/workspace/model"
    # scp model/cls.onnx root@172.20.10.9:/home/root/workspace/model/cls.onnx
    # scp model/class_names.json root@172.20.10.9:/home/root/workspace/model/class_names.json
    if ! scp main root@192.168.1.201:/home/root/workspace; then
        echo "[警告] main 上传失败，已保留本地构建产物。"
    fi
    if ! ssh root@192.168.1.201 "mkdir -p /home/root/workspace/model"; then
        echo "[警告] 远端 model 目录创建失败，已保留本地构建产物。"
    fi
    if ! scp model/cls.onnx root@192.168.1.201:/home/root/workspace/model/cls.onnx; then
        echo "[警告] cls.onnx 上传失败，已保留本地构建产物。"
    fi
    if ! scp model/class_names.json root@192.168.1.201:/home/root/workspace/model/class_names.json; then
        echo "[警告] class_names.json 上传失败，已保留本地构建产物。"
    fi
    if ! scp model/deploy_calibration.json root@192.168.1.201:/home/root/workspace/model/deploy_calibration.json; then
        echo "[警告] deploy_calibration.json 上传失败，已保留本地构建产物。"
    fi
    if [ -d model_mlp_wider_grayred_taskroi320_realcal_synsel_ls005_v1 ]; then
        if ! ssh root@192.168.1.201 "mkdir -p /home/root/workspace/model_mlp_wider_grayred_taskroi320_realcal_synsel_ls005_v1"; then
            echo "[警告] 远端 grayred32 模型目录创建失败，已保留本地构建产物。"
        fi
        if ! scp model_mlp_wider_grayred_taskroi320_realcal_synsel_ls005_v1/cls.onnx root@192.168.1.201:/home/root/workspace/model_mlp_wider_grayred_taskroi320_realcal_synsel_ls005_v1/cls.onnx; then
            echo "[警告] grayred32 cls.onnx 上传失败，已保留本地构建产物。"
        fi
        if ! scp model_mlp_wider_grayred_taskroi320_realcal_synsel_ls005_v1/class_names.json root@192.168.1.201:/home/root/workspace/model_mlp_wider_grayred_taskroi320_realcal_synsel_ls005_v1/class_names.json; then
            echo "[警告] grayred32 class_names.json 上传失败，已保留本地构建产物。"
        fi
        if ! scp model_mlp_wider_grayred_taskroi320_realcal_synsel_ls005_v1/deploy_calibration.json root@192.168.1.201:/home/root/workspace/model_mlp_wider_grayred_taskroi320_realcal_synsel_ls005_v1/deploy_calibration.json; then
            echo "[警告] grayred32 deploy_calibration.json 上传失败，已保留本地构建产物。"
        fi
    fi
fi
popd
