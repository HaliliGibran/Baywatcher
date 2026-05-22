# 2026-04-19

## 运行板 OpenCV 构建失败排查 - 第1步

- 尝试/操作
  - 读取根目录开发日志 `ai工作日志.md`，确认当前日志文件可继续追加。

- 结果
  - 日志文件当前为空，可直接作为统一开发记录入口继续使用。

- 已解决
  - 确认了后续排查记录的落点，不需要新建开发文档。

- 未解决
  - 运行板当前 `opencv2/core/cvdef.h` 缺失的编译错误还未排查。

- 新问题
  - 无。

- 后续方向
  - 直接检查运行板 `cross_lib/opencv` 目录是否仍为残包，并据此决定修复方式。

## 运行板 OpenCV 构建失败排查 - 第2步

- 尝试/操作
  - 检查运行板 `cross_lib/opencv/include/opencv4/opencv2/core`
  - 检查运行板 `cross_lib/opencv/include/opencv4/opencv2/dnn`
  - 检查运行板 `cross_lib/opencv/lib`
  - 读取运行板 `CMakeLists.txt`

- 结果
  - `core` 和 `dnn` 目录下只看到了子目录，没有看到 `cvdef.h`、`dnn.hpp` 这类核心头文件
  - `lib` 目录里有一部分 `.so`，但头文件树明显不完整
  - 运行板 `CMakeLists.txt` 当前仍然强依赖工程内 `cross_lib/opencv`

- 已解决
  - 确认运行板当前报错不是源码逻辑问题，而是运行板自己的 OpenCV 头文件包不完整
  - 确认修复方向应优先放在构建侧或补全运行板 OpenCV 包

- 未解决
  - 还没有实际修改运行板 `CMakeLists.txt`
  - 还没有让运行板构建摆脱这份残缺的 `cross_lib/opencv`

- 新问题
  - 运行板和识别板的 OpenCV 资源状态不一致：识别板之前已恢复过完整 OpenCV，而运行板当前看起来仍是残包

- 后续方向
  - 在不动控制侧代码的前提下，优先把运行板 `CMakeLists.txt` 改成可接受显式 OpenCV 根目录或系统 OpenCV 的构建方式
  - 如果用户更偏向直接拷包，也可以把识别板那份完整 OpenCV 同步到运行板

## 运行板 OpenCV 构建失败排查 - 第3步

- 尝试/操作
  - 修改运行板 `CMakeLists.txt`
  - 为 OpenCV 增加多级回退逻辑

- 修改内容
  - 新增 `LQ_OPENCV_ROOT` 可选缓存变量
  - 新增检测函数 `lq_try_use_opencv_root(...)`
  - OpenCV 查找顺序改为：
    1. 用户显式指定的 `LQ_OPENCV_ROOT`
    2. 运行板本地 `cross_lib/opencv`
    3. 识别板兄弟工程 `../lq_2k0301_project_template_for_recognition/cross_lib/opencv`
    4. `find_package(OpenCV ...)`
  - 只有检测到 `cvdef.h`、`dnn.hpp` 和 `lib` 同时存在时，才会把该目录当成有效 OpenCV 根目录使用

- 解决了什么
  - 运行板构建不再被自己这份残缺 `cross_lib/opencv` 直接卡死
  - 如果识别板那边已有完整 OpenCV，运行板现在可以直接复用

- 还没解决什么
  - 还没有实际重新编译验证
  - 如果系统 OpenCV 架构不对，`find_package(OpenCV ...)` 路径仍可能不适合交叉编译

- 新问题
  - 无新增源码问题；当前风险主要在“最终选中的 OpenCV 是否与目标架构匹配”

- 后续方向
  - 让用户重新执行 `bash build.sh`
  - 如果继续报 OpenCV 相关错误，再看 CMake 最终选中了哪一套 OpenCV

## 运行板编译失败排查 - 第4步

- 尝试/操作
  - 读取运行板 `main.cpp`
  - 读取运行板 `vision_runtime.cc/.h`
  - 读取运行板 `image_data.h/.cc`
  - 读取运行板 `TargetHandler.h/.cc`

- 结果
  - 运行板当前磁盘现状不是纯双板版本，而是“旧单板本地识别逻辑 + 新双板碎片”混合状态
  - `vision_runtime.cc` 仍然包含 `#include "recognition_chain.h"`，但运行板目录下其实已经没有这个头文件
  - `main.cpp` 仍在解析 `recognition` 开关，并按双参数调用 `Vision_System_Run(...)`
  - `image_data.cc` 顶部实际生效的代码只保留了基础图像状态，全套远端事件适配层被注释掉了
  - `TargetHandler` 当前已经是“阶段时序保留、输出改为 pure_angle 接管”的版本，可以直接复用，不需要改控制侧实现

- 已解决
  - 明确了下一步修复范围应限制在：
    - `vision_runtime.*`
    - `image_data.*`
    - `main.cpp`
  - 不需要去补一个假的 `recognition_chain.h`

- 未解决
  - 运行板当前还不能编过

- 新问题
  - 运行板源码状态与之前口头交接不一致，后续必须以当前磁盘事实为准继续修

- 后续方向
  - 只把运行板图像侧和通信接线重新收口到双板版本
  - 明确遵守用户新约束：不改控制侧代码

## 运行板双板事件适配层恢复

- 尝试/操作
  - 修改 `Code/User/image/image_data.h`
  - 修改 `Code/User/image/image_data.cc`

- 修改内容
  - 在 `image_data.h` 新增远端事件接口声明：
    - `image_remote_recognition_reset()`
    - `image_remote_recognition_apply_event(...)`
    - `image_remote_recognition_try_get_hold_yaw(...)`
  - 在 `image_data.cc` 恢复远端事件运行时缓存：
    - 上一事件 `seq` 去重
    - `vehicle` 保持航向窗口
    - `weapon/supply` 触发本地 `TargetHandler`
  - 复位时同步清理 `recognition_follow_override`

- 解决了什么
  - 运行板重新具备了“双板识别事件 -> 本地动作状态”这一层 owner
  - 后续 `vision_runtime.cc` 可以不再依赖旧 `RecognitionChain`

- 未解决
  - `vision_runtime.cc` 和 `main.cpp` 还没收口
  - 编译还不能通过

- 新问题
  - 无新增控制侧问题；当前修改只使用既有 `TargetHandler` 接口，没有改其实现

- 后续方向
  - 重写运行板 `vision_runtime.cc`
  - 同步修正 `main.cpp` 的启动入口与串口线程接线

## 运行板 vision_runtime/main 重新收口到双板版本

- 尝试/操作
  - 重写 `Code/User/image/vision_runtime.cc`
  - 修改 `Code/main.cpp`

- 修改内容
  - `vision_runtime.cc`
    - 删除对不存在的 `recognition_chain.h` 的依赖
    - 顶层改成固定灰度采集 `camera.capture_frame(..., IMREAD_GRAYSCALE, ...)`
    - 只保留三条分支：
      - 本地绕行动作 pure_angle 接管
      - 远端 `vehicle` 保持航向
      - 普通巡线
    - 普通巡线分支继续复用原有：
      - OTSU
      - 开闭运算
      - `img_processing(binimg)`
      - 图传画边线/路径
  - `main.cpp`
    - 去掉 `#include "recognition_chain.h"`
    - 恢复 `comm.init(UART1, B115200)`
    - 启动时调用 `image_remote_recognition_reset()`
    - 恢复 `task_board_comm_rx(...)`
    - 启动 `board_comm_thread`
    - 去掉 `recognition` 开关解析
    - 主入口改回 `Vision_System_Run(stream_enabled)`

- 解决了什么
  - 解决了当前编译阶段 `vision_runtime.cc` 找不到 `recognition_chain.h` 的直接错误
  - 运行板主入口和视觉运行时重新与双板方案一致

- 未解决
  - 还没有重新编译验证
  - 后面可能还会暴露新的签名或链接错误

- 新问题
  - 无新增控制侧变更；本轮没有修改 PID 或 TargetHandler 实现

- 后续方向
  - 让用户重新编译
  - 根据新的第一处报错继续收口运行板剩余残留
