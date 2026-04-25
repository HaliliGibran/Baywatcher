# 运行板图像与通信链路说明

本文档只描述当前运行板事实。

当前运行板职责固定为：
- 灰度巡线
- 元素状态机
- 图传
- VOFA 与控制协同
- UART1 接收识别板状态流

运行板已经删除本地识别主链，不再加载模型，不再解析 `--recognition`，也不再在本板做红色触发和分类。

## 1. 主入口

主入口文件：
- `Code/main.cpp`
- `Code/main.hpp`

`main()` 的执行顺序：
1. 加载板级驱动模块
2. 调用 `system_init()`
3. 启动控制、图像相关线程
4. 将终端设为非阻塞
5. 解析图传开关 `--stream`
6. 阻塞进入 `Vision_System_Run(stream_enabled)`

`system_init()` 当前固定初始化：
- 巡线相机：`160x120`
- 巡线帧率：`vision_runtime::kLineFrameFps`
- 双板通信：`comm.init(UART1, B115200)`
- 启动前清空远端识别状态缓存：`image_remote_recognition_reset()`

## 2. 线程关系

运行板当前和图像/双板协同直接相关的线程：

- `TimerThread pid_thread(BayWatcher_Control_Loop, ..., 5)`
  - 控制主环
- `TimerThread turn_thread(BayWatcher_Cube_Loop, ..., 10)`
  - 图像输出到控制修正量的外环
- `TimerThread board_comm_thread(task_board_comm_rx, ..., 5)`
  - 串口收识别板状态包
- `TimerThread vofa_thread(task_vofa_comm, ..., 20)`
  - VOFA 数据收发

说明：
- 控制侧 owner 仍在 `PID.cc`
- 本次运行板清理没有改控制算法，只清理本地识别残留并统一通信

## 3. 视觉顶层调度

顶层文件：
- `Code/User/image/vision_runtime.h`
- `Code/User/image/vision_runtime.cc`

当前视觉运行时固定为单入口：

```cpp
void Vision_System_Run(bool stream_enabled);
```

运行时主循环每帧流程：
1. 读取键盘，检查是否按下 `c`
2. 固定按灰度采集：
   - `camera.capture_frame(img, true, cv::IMREAD_GRAYSCALE, false)`
3. 更新时间戳
4. 调用 `handler_sys.Update_Tick()`
5. 按优先级进入三个分支之一

分支优先级：
1. 绕行动作接管分支
2. `vehicle` 保持当前航向分支
3. 普通巡线分支

## 4. 手动复位

按键：
- `c`
- `C`

执行位置：
- `vision_runtime.cc / HandleManualVisionReset()`

当前复位动作：
- `track_force_reset()`
- `handler_sys.Stop_Action()`
- `image_remote_recognition_reset()`

含义：
- 清空图像侧元素状态机
- 强制停止当前绕行动作
- 清空远端状态去重和 `vehicle hold` 缓存

## 5. 普通巡线分支

普通巡线渲染与处理在：
- `vision_runtime.cc / RenderLineTrackingView(...)`

每帧处理顺序：
1. 若输入不是 `160x120`，先缩放到巡线工作分辨率
2. 若输入不是单通道，转灰度
3. Otsu 二值化
4. 开运算
5. 闭运算
6. 清零四周边框
7. 调用 `img_processing(binimg)`

当前普通巡线图传支持两种底图：
- `BW_STREAM_LINE_USE_BINARY_VIEW = 1`
  - 图传显示二值图
- `BW_STREAM_LINE_USE_BINARY_VIEW = 0`
  - 图传显示灰度底图

随后叠加：
- 左右边线点
- 十字远端线点
- `midline.path`

## 6. 图像主链 owner

图像主链入口：
- `Code/User/image/image_process.cc`

核心职责：
- 左右边线搜索
- 单边边线处理
- 环岛/十字/斑马线状态机
- 中线融合
- 路径生成
- `pure_angle` 计算

相关状态定义：
- `Code/User/image/image_data.h`
- `Code/User/image/image_data.cc`

主要全局状态：
- `pts_left / pts_right`
- `pts_far_left / pts_far_right`
- `if_find_far_line`
- `midline`
- `follow_mode`
- `element_type`
- `circle_state`
- `circle_direction`
- `crossing_state`
- `pure_angle`
- `average_curvature`
- `preview_img_y`
- `zebra_stop`

## 7. 元素状态机

元素总入口：
- `Code/User/image/element.cc`

环岛状态机：
- `Code/User/image/element/circle.cc`

十字状态机：
- `Code/User/image/element/crossing.cc`

斑马线检测：
- `Code/User/image/element/zebra.cc`

运行板当前仍保留完整元素状态机主链。
双板化后变动的是“识别动作从本地模型触发”改成了“来自识别板串口状态流”，不是改元素链本身。

## 8. 中线、路径与 pure_angle

相关文件：
- `Code/User/image/image_handle.cc`
- `Code/User/image/image_midline_process.cc`

当前行为：
- 从左右边线生成候选中线
- 根据 `follow_mode` 融合最终中线
- 从车体 core 点并轨到中线生成 `midline.path`
- 使用“车轴到预瞄点的割线角”计算 `pure_angle`

当前仍保留的关键观测量：
- `pure_angle`
- `average_curvature`
- `preview_img_y`

这些量会被 VOFA 发送，用于调参和看状态。

## 9. 运行板与识别板协同

当前运行板不再有本地识别链。
识别板只通过串口发送状态码，运行板只消费状态码。

状态协议定义位置：
- 运行板：`Code/User/inc/Communication.h`
- 识别板：`Code/User/inc/Communication.h`

两板当前已经对齐为同一套状态协议：
- 帧头：`0x5A 0xA5`
- 版本：`0x01`
  - 状态码枚举：
  - `WEAPON`
  - `SUPPLY`
  - `VEHICLE`
- CRC8
- 帧尾：`0xED`

串口配置：
- 端口：`UART1`
- 设备：`/dev/ttyS1`
- 波特率：`B115200`

## 10. 运行板通信链

文件：
- `Code/User/inc/Communication.h`
- `Code/User/src/Communication.cc`

当前通信职责：
- `BoardComm::init(...)`
  - 初始化 `UART1@115200`
- `BoardComm::try_receive_state(...)`
  - 从串口字节流中重组状态帧
- `BoardComm::send_state(...)`
  - 当前运行板主流程不主动发送识别状态，但接口与识别板保持一致

串口接收线程：
- `main.cpp / task_board_comm_rx(...)`

处理逻辑：
1. `comm.try_receive_state(...)`
2. 若收到合法新状态包
3. 调用 `image_remote_recognition_apply_state(...)`

## 11. 远端识别状态适配层

文件：
- `Code/User/image/image_data.h`
- `Code/User/image/image_data.cc`

当前运行板只保留远端状态适配层，不再保留本地识别覆盖逻辑。

适配层职责：
- 状态序号去重
- 忙碌时丢弃新的动作状态
- `vehicle` 状态锁存当前 `pure_angle` 一段时间
- `weapon/supply` 状态触发本地绕行动作

关键接口：
- `image_remote_recognition_reset()`
- `image_remote_recognition_apply_state(...)`
- `image_remote_recognition_try_get_hold_yaw(...)`

当前具体语义：
- `WEAPON`
  - 映射为 `TargetBoardType::WEAPON`
- `SUPPLY`
  - 映射为 `TargetBoardType::SUPPLIES`
- `VEHICLE`
  - 锁存状态发生瞬间的 `pure_angle`
  - 持续时间由 `BW_REMOTE_VEHICLE_HOLD_MS` 控制

忙碌判定：
- `handler_sys.Is_Executing()`
- 或仍在 `vehicle hold` 窗口内

只要处于忙碌态，新的动作状态直接丢弃，不排队。

## 12. 绕行动作如何接回控制侧

当前运行板没有再走“识别结果强制左线/右线巡线”的旧方案。

现在行为是：
- 识别板发来 `WEAPON/SUPPLY`
- 运行板状态适配层调用 `handler_sys.Start_Action(...)`
- `vision_runtime.cc` 每帧先 `handler_sys.Update_Tick()`
- 若 `handler_sys.TryGetPureAngleOverride(...)` 返回有效
- 本帧直接由绕行动作接管 `pure_angle`

这条链路的 owner：
- 动作状态：`TargetHandler`
- 顶层调度：`vision_runtime.cc`
- 控制消费：`PID.cc`

## 13. 图像侧与控制侧协同

控制侧没有被这轮清理改动。

协同方式仍然是：
1. 图像侧输出 `pure_angle`
2. 控制侧读取 `pure_angle`
3. 控制侧在 `BayWatcher_Cube_Loop` / `BayWatcher_Control_Loop` 中继续完成控制计算

当前运行板双板化后，图像侧对控制侧新增的只是一条“远端状态可以接管 `pure_angle`”的上层分支，不是重写底层控制。

## 14. 图传

图传链文件：
- `Code/User/image/stream_chain.h`
- `Code/User/image/stream_chain.cc`

当前图传行为：
- 编译期默认开关：`BW_ENABLE_STREAM`
- 运行时开关：
  - `--stream`
  - `--no-stream`
  - `--stream=on|off`
  - `--stream-mode on|off`

运行板图传关闭时：
- 仍做正常巡线处理
- 不构建调试 `view`
- 不做额外灰度转 BGR 与画线叠加

运行板图传开启时：
- 普通巡线态显示灰度/二值底图 + 边线 + 路径
- 绕行态显示 `REMOTE BYPASS: PURE_ANGLE`
- `vehicle hold` 态显示 `REMOTE VEHICLE: HOLD YAW`

## 15. 关键参数位置

图像与双板协同参数主要集中在：
- `Code/User/image/common.h`

当前和双板运行板最相关的参数：
- `BW_ENABLE_STREAM`
- `BW_STREAM_LINE_USE_BINARY_VIEW`
- `BW_REMOTE_VEHICLE_HOLD_MS`
- `BW_BYPASS_LEFT_*`
- `BW_BYPASS_RIGHT_*`
- `BW_GRAY_BIN_DIAG_*`

说明：
- 本地识别触发、模型识别、ROI、类别表等参数已经不再属于运行板
- 这些参数应只在识别板维护

## 16. 清理结果

本轮运行板清理后的状态：
- 已删除本地 `ncnn` 封装源码
- 已删除运行板本地识别宏残留
- 已删除运行板本地识别覆盖状态定义
- 已统一到 `UART1@115200`
- 已把 `BoardComm` 的实现与识别板协议对齐
- 已把全局 `comm` owner 收回 `Communication.cc`

仍保留但属于正常运行板资产的内容：
- 巡线图像主链
- 元素状态机
- TargetHandler 动作链
- 图传链
- VOFA 链
