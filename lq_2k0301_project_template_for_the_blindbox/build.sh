#!/bin/bash

set -e

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
    if ! scp main root@192.168.1.203:/home/root/workspace; then
        echo "[警告] main 上传失败，已保留本地构建产物。"
    fi
fi

popd
