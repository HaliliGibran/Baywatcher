#!/bin/bash

# 该指令表示任意指令执行失败，立即终止脚本
set -e

# # 默认开启图传
# CMAKE_STREAM_OPT="-DBW_ENABLE_STREAM=ON"

# # 如果运行脚本时带有 "off" 参数，则关闭图传
# if [ "$1" == "off" ]; then
#     echo -e "\n[配置] 检测到参数 'off' -> 图传功能已关闭"
#     CMAKE_STREAM_OPT="-DBW_ENABLE_STREAM=OFF"
# else
#     echo -e "\n[配置] 默认编译 -> 图传功能已开启"
# fi

echo -e "\n=== 构建 build目录 ==="
cmake -B output
# cmake -B output $CMAKE_STREAM_OPT

pushd output
make -j$(nproc)

if [ -f main ]; then
	echo -e "\n===== 编译成功 ====="
	#这里可以改成scp传输到我们的板卡上
	scp main root@172.20.10.9:/home/root/workspace
fi
popd
