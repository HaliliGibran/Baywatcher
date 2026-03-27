# 运行板图像与控制链说明

本文档描述当前双板方案下运行板的真实主流程，只以当前主入口和默认构建为准。

当前运行板职责固定为：

- 固定灰度相机采集 `160x120@110`
- 二值化、寻线、元素检测、中线路径构建、`pure_angle` 输出
- 串口接收识别板事件，并映射为本地 `vehicle` 保持航向 / 左右绕行动作
- 控制线程消费 `pure_angle`，输出 `speed_adjust`、左右轮速度与 PWM
- 图传、VOFA、状态调试

当前运行板不负责：

- 本地彩色识别
- 红框触发
- ONNX 分类
- 本地 `RecognitionChain` 主流程调度

仓库里仍保留了历史 `recognition_chain.*`，但它已经不是当前运行板主链 owner，也不属于默认构建主路径。

## 1. 总体链路

运行板顶层执行关系如下：

`main.cpp`
-> `system_init()`
-> 启动控制/传感/VOFA/串口收包线程
-> 进入 `Vision_System_Run(stream_enabled)`
-> 图像主循环输出 `pure_angle`
-> `BayWatcher_Cube_Loop` 把 `pure_angle` 转成 `PID.speed_adjust`
-> `BayWatcher_Control_Loop` 用 `speed_adjust + yaw_control_base_speed` 输出左右轮目标速度和 PWM

双板协同时序如下：

1. 识别板通过 `UART1@115200` 发送一次性事件。
2. 运行板 `task_board_comm_rx` 调 `comm.try_receive_event(...)` 收事件。
3. 收到事件后，统一喂给 `image_remote_recognition_apply_event(...)`。
4. `vehicle` 事件锁存当前 `pure_angle` 一段时间。
5. `weapon/supply` 事件触发本地 `TargetHandler` 绕行状态机。
6. `vision_runtime` 在每帧顶层按优先级决定本帧由谁接管 `pure_angle`。

## 2. 入口文件与执行关系

### 2.1 主入口

主入口在 `Code/main.cpp`，负责：

- 硬件模块加载
- `system_init()` 初始化
- 创建并启动线程
- 解析图传开关
- 进入 `Vision_System_Run(stream_enabled)`

当前 `system_init()` 做的关键初始化：

- `camera.init(160, 120, 110)`
- `comm.init(UART1, B115200)`
- `BayWatcher_Control_Init()`
- `handler_sys.init()`
- `image_remote_recognition_reset()`

当前启动的线程及作用：

| 线程 | 周期 | 作用 |
| --- | --- | --- |
| `BayWatcher_Control_Loop` | 2ms | 底层速度闭环，输出电机 PWM |
| `BayWatcher_Cube_Loop` | 10ms | 读取 `pure_angle`，计算 `PID.speed_adjust` |
| `task_sensor_read` | 20ms | 按键/传感器轮询 |
| `task_vofa_comm` | 20ms | VOFA 收发与关键观测量上报 |
| `task_board_comm_rx` | 5ms | 串口收双板事件 |

注意：

- 当前只启动一次 `pid_thread.start()`，避免重复启动同一控制线程。
- 运行板当前只解析图传开关，不再解析本地识别开关。
- 终端会被设成非阻塞，用于按键 `c` 手动复位。

### 2.2 图像运行时入口

图像主循环入口在 `Code/User/image/vision_runtime.h/.cc`：

- `kLineFrameWidth = 160`
- `kLineFrameHeight = 120`
- `kLineFrameFps = 110`
- `void Vision_System_Run(bool stream_enabled);`

顶层循环每帧顺序固定为：

1. `HandleManualVisionReset()`
2. 固定灰度抓帧 `camera.capture_frame(..., IMREAD_GRAYSCALE, ...)`
3. `handler_sys.Update_Tick()`
4. 优先执行绕行接管分支
5. 其次执行 `vehicle` 保持航向分支
6. 否则进入普通巡线分支
7. `stream.PublishFrame(view)`

分支优先级固定为：

1. `TargetHandler` 绕行 `pure_angle` 接管
2. 远端 `vehicle` 保持航向
3. 普通巡线图像链

## 3. 文件 owner 一览

| 文件 | 当前 owner |
| --- | --- |
| `Code/main.cpp` | 主入口、线程关系、硬件初始化 |
| `Code/User/image/vision_runtime.cc` | 图像主循环总调度、图传显示分支 |
| `Code/User/image/image_process.cc` | 二值图主处理总控 |
| `Code/User/image/image_handle.cc` | 寻线、单侧边线流水线、`pure_angle` 计算 |
| `Code/User/image/image_midline_process.cc` | 中线融合、路径构建、动态预瞄建议 |
| `Code/User/image/element.cc` | 元素判定入口与元素 owner 切换 |
| `Code/User/image/element/circle.cc` | 环岛状态机 |
| `Code/User/image/element/crossing.cc` | 十字状态机与远端线补线 |
| `Code/User/image/element/zebra.cc` | 斑马线检测 |
| `Code/User/image/image_data.h/.cc` | 图像侧全局状态、远端事件适配层 |
| `Code/User/src/PID.cc` | 控制闭环、曲率减速、`pure_angle -> speed_adjust` |
| `Code/User/src/TargetHandler.cc` | 双板绕行动作状态机 |
| `Code/User/inc/Communication.h` + `Code/User/src/Communication.cc` | 双板事件协议与串口收发 |
| `Code/User/image/stream_chain.cc` | 图传开关解析、服务启动、降频发布 |

## 4. 参数和开关在哪

### 4.1 图像与状态机参数

主要在 `Code/User/image/common.h`。

关键分区如下：

- `图像基础参数`
  - `IMAGE_H`
  - `IMAGE_W`
  - `PIXPERMETER`
  - `ROADWIDTH`
  - `PT_MAXLEN`
- `图像预处理与寻线基础参数`
  - `SET_IMAGE_CORE_X`
  - `SET_IMAGE_CORE_Y`
  - `SEARCH_LINE_START_OFFSET`
  - `SELFADAPT_KERNELSIZE`
  - `FILTER_KERNELSIZE`
  - `RESAMPLEDIST`
  - `ANGLEDIST`
- `pure_angle预瞄与路径参数`
  - `PUREANGLE_PREVIEW_BASE_IMAGE_Y`
  - `PUREANGLE_PREVIEW_MIN_IMAGE_Y`
  - `PUREANGLE_PREVIEW_CURV_DIST`
  - `PUREANGLE_PREVIEW_CURV_NMS_KERNEL`
  - `PUREANGLE_PREVIEW_CURVE_LOW`
  - `PUREANGLE_PREVIEW_CURVE_HIGH`
  - `PUREANGLE_PREVIEW_SHIFT_MAX`
  - `PATH_BLEND_REF_IMAGE_Y`
- `pure_angle预瞄过渡参数`
  - `PUREANGLE_PREVIEW_TRANSITION_ENABLE`
  - `PUREANGLE_PREVIEW_TRANSITION_MAX_STEP`
  - `PUREANGLE_PREVIEW_TRANSITION_ALPHA`
- `pure_angle丢线补偿参数`
  - `PUREANGLE_LOST_TREND_ENABLE`
  - `PUREANGLE_LOST_TREND_FRAMES`
  - `PUREANGLE_LOST_MAX_STEP_DEG`
  - `PUREANGLE_LOST_RATE_DECAY`
  - `PUREANGLE_LOST_RETURN_ZERO_DECAY`
  - `PUREANGLE_REACQ_BLEND_FRAMES`
- `pure_angle趋势前馈参数`
  - `PUREANGLE_PRE_CTRL_ENABLE`
  - `PUREANGLE_PRE_CTRL_START_DEG`
  - `PUREANGLE_PRE_CTRL_DELTA_START_DEG`
  - `PUREANGLE_PRE_CTRL_GAIN`
  - `PUREANGLE_PRE_CTRL_MAX_EXTRA_DEG`
- `斑马线检测与停车参数`
  - `ZEBRA_STEP`
  - `NEAR_DETECT_MAX`
  - `ZEBRA_MIN_RUNS`
  - `BW_ENABLE_ZEBRA_RUN_LENGTH_CHECK`
  - `ZEBRA_RUN_DIFF_RATIO`
  - `ZEBRA_RUN_SPREAD_RATIO`
  - `ZEBRA_COOLDOWN_MS`
  - `ZEBRA_STOP_DELAY_MS`
- `双板通信与绕行动作参数`
  - `BW_REMOTE_VEHICLE_HOLD_MS`
  - `BW_BYPASS_LEFT_*`
  - `BW_BYPASS_RIGHT_*`
- `图传开关与图传模式切换`
  - `BW_ENABLE_STREAM`
  - `BW_STREAM_LINE_USE_BINARY_VIEW`
- `元素判定参数`
  - `ID_THRESHOLD_crossing_state_change`
  - `FRAME_THRESHOLD_roundabout_protect_frame`
  - `FRAME_THRESHOLD_roundabout_or_crossing_frame`
  - `FRAME_THRESHOLD_one_corner_crossing_protect_frame`
  - `WINDOW_THRESHOLD_roundabout_opposite_straightness`
- `元素状态机帧阈值参数`
  - `FRAME_THRESHOLD_crossing_lost_line_counter`
  - `FRAME_THRESHOLD_roundabout_begin_to_in_lost_line_counter`
  - `FRAME_THRESHOLD_roundabout_begin_to_in_found_line_counter`
  - `FRAME_THRESHOLD_roundabout_running_to_out_corner_counter`
  - `FRAME_THRESHOLD_roundabout_end_lost_line_counter`
  - `FRAME_THRESHOLD_roundabout_end_found_line_counter`

注意：

- `common.h` 里仍有一组“历史保留: 本地识别链与图传默认参数”，那是给历史文件参考，不是当前运行板主流程参数入口。

### 4.2 控制参数

主要在 `Code/User/src/PID.cc`。

当前最重要的是曲率减速参数：

- `BW_ENABLE_CURVE_SLOWDOWN`
- `BW_ENABLE_ROUNDABOUT_CURVE_SLOWDOWN`
- `BW_ROUNDABOUT_FIXED_SLOWDOWN_RATIO`
- `BW_CURVE_SLOWDOWN_CURV_LOW`
- `BW_CURVE_SLOWDOWN_CURV_HIGH`
- `BW_CURVE_SLOWDOWN_MIN_SPEED_RATIO`
- `BW_CURVE_SLOWDOWN_MIN_SPEED_ABS`
- `BW_CURVE_SLOWDOWN_ALPHA_DOWN`
- `BW_CURVE_SLOWDOWN_ALPHA_UP`

### 4.3 运行时开关

运行时命令行开关在 `stream_chain.cc`：

- `--stream`
- `--no-stream`
- `--stream=on|off`
- `--stream-mode on|off`

当前运行板没有本地识别开关。

## 5. 关键状态量

关键全局状态量集中在 `Code/User/image/image_data.h/.cc`。

### 5.1 边线与中线相关

- `pts_left` / `pts_right`
  - 左右边线完整处理上下文
  - 包含原始点、逆透视点、滤波点、重采样点、曲率、角点、单侧中线
- `pts_far_left` / `pts_far_right`
  - 十字远端线专用缓存
- `if_find_far_line`
  - 当前十字是否找到了远端补线
- `midline`
  - `midline.mid`
  - `midline.path`

### 5.2 状态机相关

- `follow_mode`
  - `MIXED`
  - `MIDLEFT`
  - `MIDRIGHT`
- `element_type`
  - `NORMAL`
  - `CIRCLE`
  - `CROSSING`
- `circle_state`
  - `CIRCLE_NONE`
  - `CIRCLE_BEGIN`
  - `CIRCLE_IN`
  - `CIRCLE_RUNNING`
  - `CIRCLE_OUT`
  - `CIRCLE_END`
- `circle_direction`
  - `LEFT`
  - `RIGHT`
- `crossing_state`
  - `CROSSING_NONE`
  - `CROSSING_IN`
  - `CROSSING_RUNNING`
- `zebra_stop`
  - 图像侧斑马线停车请求标志

### 5.3 观测量与输出量

- `pure_angle`
  - 图像侧最终输出角度，右转负、左转正
- `circle_average_angle`
  - 环岛 `RUNNING` 阶段的平均角缓存
- `average_curvature`
  - 动态预瞄使用的平均曲率观测量
- `preview_img_y`
  - 当前帧实际预瞄图像行
- `g_track_debug`
  - `crossing_vote`
  - `circle_vote`
  - `protect`

### 5.4 远端事件相关

运行板远端事件缓存不对外暴露结构体，只通过接口访问。内部实际维护：

- 最近一次 `seq` 是否有效
- 最近一次 `seq`
- `vehicle` 保持航向截止时间
- `vehicle` 锁存的 `pure_angle`

## 6. 普通巡线态完整链路

普通巡线分支主链在 `vision_runtime.cc -> RenderLineTrackingView(...) -> img_processing(binimg)`。

完整链路如下：

1. 固定采灰度图 `160x120`
2. `OTSU` 二值化
3. `3x3` 开运算
4. `3x3` 闭运算
5. 清零四周边框
6. `img_processing(binimg)` 进入图像主链

`img_processing(...)` 内部顺序固定为：

1. `handle_zebra_stop_lifecycle(t_ms)`
2. `process_track_edges(img)`
3. `try_trigger_zebra_pending_stop(img, t_ms)`
4. `update_track_state_machine(img)`
5. `build_midline_from_current_state()`
6. `build_path_and_measure_pure_angle()`
7. `finalize_pure_angle_output(...)`
8. `pure_angle_apply_pre_control(...)`

其中单侧边线流水线由 `process_line(...)` 封装，实际包含：

- 逆透视
- 滤波
- 重采样
- 曲率计算
- 角点检测
- 单侧中线生成

## 7. 元素检测与状态机链路

元素入口在 `Code/User/image/element.cc` 的 `element_detect()`。

当前元素判定思路：

1. 先读左右边线角点结果
2. 生成十字候选和环岛候选
3. 用投票计数和保护帧做抗抖
4. 用优先级锁定防止低优先级状态覆盖高优先级正在运行的状态机

当前优先级固定为：

`CROSSING > CIRCLE > NORMAL`

判定核心：

- 十字候选
  - 左右都有角点
  - 两侧 `corner_id` 都小于 `ID_THRESHOLD_crossing_state_change`
- 环岛候选
  - 只有单侧有角点
  - 对侧在局部窗口里更像直线

特殊复位：

- 如果已经处于环岛/十字，但两侧边线都重新变成大直线，会强制复位到 `NORMAL`

`track_force_reset()` 的作用：

- 元素 owner 直接收回 `NORMAL`
- `follow_mode` 回 `MIXED`
- 清空十字远端线
- 复位环岛/十字状态机内部计数
- 请求 `element_detect()` 内部投票与保护帧清零

## 8. 环岛状态机

环岛状态机在 `Code/User/image/element/circle.cc`。

状态含义：

- `CIRCLE_BEGIN`
  - 刚确认环岛，先贴外侧跟线
- `CIRCLE_IN`
  - 入环，切到内侧中线
- `CIRCLE_RUNNING`
  - 环内运行，再次贴外侧更稳
- `CIRCLE_OUT`
  - 检测到出环特征后切回内侧中线准备出环
- `CIRCLE_END`
  - 等待“丢线 -> 找线”完成，正式退出环岛

左右环岛跟线关系：

- 右环岛
  - `BEGIN`: `MIDLEFT`
  - `IN`: `MIDRIGHT`
  - `RUNNING`: `MIDLEFT`
  - `OUT`: `MIDRIGHT`
  - `END`: `MIDLEFT`
- 左环岛
  - 对称镜像

状态推进依据：

- `BEGIN -> IN`
  - 外侧连续丢线后再连续找线
- `IN -> RUNNING`
  - 对侧出现弯曲边线
- `RUNNING -> OUT`
  - 对侧角点连续出现
- `OUT -> END`
  - 对侧重新变直
- `END -> NONE`
  - 外侧再次完成“丢线 -> 找线”

图像侧对环岛还有两条协同逻辑：

- `circle_average_angle`
  - 在 `RUNNING` 阶段累计平均角
- 出环保护
  - `CIRCLE_OUT` 阶段如果跟踪侧边线丢失，会优先使用 `circle_average_angle`

## 9. 十字状态机

十字状态机在 `Code/User/image/element/crossing.cc`。

状态含义：

- `CROSSING_IN`
  - 已进入十字，但仍在边线可见阶段
- `CROSSING_RUNNING`
  - 十字中心丢线阶段

推进规则：

- `IN -> RUNNING`
  - 连续若干帧左右边线都丢失
- `RUNNING -> NONE`
  - 左右边线重新都找到

十字特殊逻辑：

- `crossing_far_line_check(img)`
  - 在十字阶段尝试寻找远端补线
- 找到远端补线后
  - `if_find_far_line = true`
  - `pts_far_left / pts_far_right` 走完整单侧流水线
  - 中线融合优先使用远端线

## 10. 斑马线逻辑

斑马线检测在 `Code/User/image/element/zebra.cc`，生命周期控制在 `image_process.cc`。

检测条件：

- 当前 `element_type == NORMAL`
- 左右边线都判成直线
- 不在冷却时间内

检测方法：

1. 沿左边线近车区域取若干扫描行
2. 从左线点向右做横向 run-length 扫描
3. 统计黑白连续色块数量
4. 若开启 `BW_ENABLE_ZEBRA_RUN_LENGTH_CHECK`
   - 进一步检查白块内部长度一致性
   - 检查黑块内部长度一致性
   - 检查白块均值与黑块均值接近

生命周期：

- 识别到斑马线
  - 进入 `pending_stop`
  - 先等待 `ZEBRA_STOP_DELAY_MS`
- 延时结束
  - 置位 `zebra_stop`
- `zebra_stop` 解除后
  - 进入 `ZEBRA_COOLDOWN_MS` 冷却

当前事实要注意：

- 图像侧确实维护了 `zebra_stop` 生命周期。
- 但 `PID.cc` 里“看到 `zebra_stop` 后直接停车”的控制接线目前是注释状态。
- 所以当前斑马线链路更准确地说是“图像侧已实现完整状态管理，控制侧停车闭环尚未正式接回”。

## 11. 中线、路径与切换线逻辑

### 11.1 跟线模式 owner

`follow_mode` 的 owner 主要是元素状态机：

- 默认 `MIXED`
- 环岛状态机会切到 `MIDLEFT` / `MIDRIGHT`
- 十字本身不直接改 `follow_mode`，而是通过远端线切换中线来源

### 11.2 中线融合逻辑

`MID(...)` 在 `image_midline_process.cc` 中负责最终中线生成。

三种模式：

- `MIXED`
  - 左右单侧中线重合段做平均
  - 点更多的一侧尾段直接接到后面
- `MIDLEFT`
  - 优先用左侧单侧中线
  - 正常赛道下左侧没有时可退回右侧
- `MIDRIGHT`
  - 对称镜像

这就是当前工程里真正的“切换线逻辑”：

- 不是单独另有一套复杂切线器
- 而是由 `follow_mode + MID(...)` 完成左右单侧中线切换和回退

### 11.3 十字远端线切换

如果 `element_type == CROSSING && if_find_far_line`：

- 最终中线不再由 `pts_left / pts_right` 融合
- 而改用 `pts_far_left / pts_far_right` 融合

### 11.4 路径构建

`BuildPathFromCoreToMidlineArc(...)` 负责从车体核心点平滑并轨到中线：

- 先找一个参考图像行 `PATH_BLEND_REF_IMAGE_Y`
- 在中线上找到接近该参考位置的目标点
- 从 core 到该目标点做渐进融合
- 后续路径与中线重合
- 最后按 `RESAMPLEDIST * PIXPERMETER` 重采样

## 12. 偏航角 / pure_angle 逻辑

`pure_angle` 计算在 `image_handle.cc` 的 `CalculatePureAngleFromPath(...)`。

当前定义不是路径切线角，而是：

- 以车轴附近点为控制参考点
- 在 `path` 上取接近某个预瞄图像行的目标点
- 用“车 -> 目标点”的割线方向计算 `pure_angle`

输出约定：

- 左转为正
- 右转为负
- 输出限幅 `[-80, 80]`

### 12.1 动态预瞄

预瞄建议逻辑在 `MidLineSuggestPureAnglePreviewImageY(...)`。

当前逻辑：

1. 对整条中线做局部曲率
2. 做 NMS
3. 只对非零峰值做均值
4. 用 `average_curvature` 在 `curve_low ~ curve_high` 间连续映射成前推量
5. 得到更远的 `preview_img_y`

之后还要过一层 `pure_angle_apply_preview_transition(...)`：

- 限制帧间最大跳变
- 再做轻滤波

### 12.2 丢线补偿

`pure_angle_apply_lost_strategy(...)` 负责：

- 短时丢线趋势外推
- 长时丢线回零
- 回线后的软切换

### 12.3 趋势前馈

`pure_angle_apply_pre_control(...)` 负责：

- 只有在“角度已经明显转起来且趋势继续增强”时才额外补一点
- 默认宏是关闭的

## 13. 绕行逻辑

绕行 owner 在 `TargetHandler`，不是图像链本身。

触发来源：

- 识别板发来 `WEAPON`
  - 运行板映射成 `TargetBoardType::WEAPON`
  - 本地执行左绕
- 识别板发来 `SUPPLY`
  - 运行板映射成 `TargetBoardType::SUPPLIES`
  - 本地执行右绕

当前阶段固定为：

- `TURN_OUT`
- `STRAIGHTEN_OUT`
- `DRIVE_PAST`
- `TURN_IN`
- `STRAIGHTEN_IN`

重要事实：

- 阶段持续帧数保留旧时序，在 `TargetHandler.cc` 里写死
- 输出量已经不是旧的差速覆盖
- 当前改成每阶段给一个 `pure_angle_override_value`
- `vision_runtime` 在绕行动作执行期间直接把 `pure_angle` 接管为这个值

当前绕行参数分两类：

- 阶段时长
  - 在 `TargetHandler.cc`
- 各阶段角度
  - 在 `common.h` 的 `BW_BYPASS_LEFT_*` / `BW_BYPASS_RIGHT_*`

## 14. 图像侧与控制侧协同逻辑

当前协同关系很明确：

1. 图像侧只负责把本帧最终 `pure_angle` 算出来
2. 绕行或 `vehicle` 需要接管时，也是在图像层把 `pure_angle` 改成接管值
3. 控制层不关心这是普通巡线角，还是远端动作接管角
4. 控制层统一只消费最终的 `pure_angle`

这意味着：

- 图像层是 `pure_angle` owner
- 控制层是 `pure_angle` consumer
- 远端事件不能直接碰 PID，只能通过 `image_data` 和 `TargetHandler` 间接影响 `pure_angle`

## 15. 控制层使用方式

当前真正在线使用的是两层：

- `BayWatcher_Cube_Loop`
  - 读取 `pure_angle`
  - 计算 `PID.speed_adjust`
- `BayWatcher_Control_Loop`
  - 根据 `PID.base_target_speed`
  - 再结合曲率减速得到 `PID.yaw_control_base_speed`
  - 用 `speed_adjust` 分配左右轮目标速度
  - 最终输出 PWM

当前对外常用接口：

- `BayWatcher_Control_Init()`
- `BayWatcher_Set_BaseSpeed(float speed)`
- `BayWatcher_Set_TargetAngle(float angle)`
- `BayWatcher_Start_Car(...)`
- `BayWatcher_Stop_Car()`

当前 `VOFA` 发的关键观测量：

- `PID.target_speed_L`
- `vL`
- `PID.pwm_out_L`
- `PID.target_speed_R`
- `vR`
- `PID.pwm_out_R`
- `pure_angle`
- `PID.yaw_control_base_speed`
- `average_curvature`
- `preview_img_y`

## 16. 图传逻辑和画线逻辑

图传链在 `stream_chain.cc`。

行为：

- `BW_ENABLE_STREAM` 给默认值
- 运行时命令行可覆盖
- 服务真正启动后，每两帧发布一次

普通巡线态图传在 `vision_runtime.cc` 中构建：

- 图传关闭时
  - 仍然跑算法
  - 不构建 `view`
  - 不做灰度转 BGR
  - 不画点
- 图传开启时
  - 可在二值底图和灰度底图间切换
  - 会叠加左右边线点
  - 十字远端线有效时改为叠加远端线点
  - 会叠加 `midline.path`

## 17. 手动复位逻辑和使用方式

### 17.1 使用方式

运行板主程序启动后，终端输入：

- `c`
- `C`

即可触发手动复位。

### 17.2 当前实际复位内容

`vision_runtime.cc` 的 `HandleManualVisionReset()` 当前会做：

- `track_force_reset()`
- `handler_sys.Stop_Action()`
- `image_remote_recognition_reset()`

也就是：

- 清元素状态机
- 清十字远端线
- 清元素投票与保护帧
- 清绕行动作状态
- 清远端 `vehicle` 保持航向缓存

### 17.3 当前不会自动清的内容

当前手动复位不会直接清：

- `PID.is_running`
- 车辆启停状态
- `zebra_stop`

也就是说：

- `c` 主要是图像态/动作态复位
- 不是整车总复位

## 18. 双板协同语义

当前双板语义固定为：

- 识别板只发事件
- 运行板只收事件并执行本地动作

事件到动作的映射：

- `WEAPON -> 左绕`
- `SUPPLY -> 右绕`
- `VEHICLE -> 保持当前航向`

运行板行为细节：

- 同一个 `seq` 只消费一次
- 动作执行中收到的新事件直接丢弃，不排队
- `vehicle` 保持时长由 `BW_REMOTE_VEHICLE_HOLD_MS` 控制

## 19. 当前不是主流程的内容

以下内容当前仍可能在仓库里存在，但不是运行板主流程：

- 历史本地 `recognition_chain.*`
- `common.h` 里历史本地识别宏
- 历史识别说明或旧注释

后续继续维护时，请以这些文件为主：

- `Code/main.cpp`
- `Code/User/image/vision_runtime.*`
- `Code/User/image/image_process.cc`
- `Code/User/image/image_handle.cc`
- `Code/User/image/image_midline_process.cc`
- `Code/User/image/element*.cc`
- `Code/User/image/image_data.*`
- `Code/User/src/PID.cc`
- `Code/User/src/TargetHandler.cc`
- `Code/User/inc/Communication.h`
- `Code/User/src/Communication.cc`

不要再按“运行板本地彩色识别”思路回退主链。
