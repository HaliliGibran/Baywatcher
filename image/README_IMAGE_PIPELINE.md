# 图像侧全链路逻辑说明

本文档基于当前工程代码整理，覆盖图像侧从采集、二值化、赛道处理、元素状态机、模型识别链路到控制输出的完整逻辑。

适用代码范围：
- `Code/main.cpp`
- `Code/User/image/vision_runtime.h`
- `Code/User/image/vision_runtime.cc`
- `Code/User/image/stream_chain.h`
- `Code/User/image/stream_chain.cc`
- `Code/User/image/recognition_chain.h`
- `Code/User/image/recognition_chain.cc`
- `Code/User/image/image_process.cc`
- `Code/User/image/image_data.h`
- `Code/User/image/image_data.cc`
- `Code/User/image/element.cc`
- `Code/User/image/element/circle.cc`
- `Code/User/image/element/crossing.cc`
- `Code/User/image/element/zebra.cc`
- `Code/User/image/image_handle.cc`
- `Code/User/image/image_midline_process.cc`
- `Code/User/image/common.h`
- `Code/User/image/image_switch_utils.h`
- `Code/User/src/PID.cc`

## 1. 总体链路

图像侧主链路分为五层：

1. `Code/main.cpp` 负责：
- 系统初始化
- 线程启动
- 命令行开关解析
- 调用 `Vision_System_Run(...)`

2. `Code/User/image/vision_runtime.cc` 负责：
- 相机取图
- 图像侧顶层调度
- 图传链、识别链、巡线链的串联
- 普通巡线态与识别态切换

3. `Code/User/image/stream_chain.cc` 负责：
- 图传开关解析
- 图传服务器启动
- 图传帧降频发布

4. `Code/User/image/recognition_chain.cc` 负责：
- 模型识别开关解析
- 识别后动作策略开关
- ONNX 模型和类别表加载
- 红色触发检测
- 识别态管理
- 投票收敛
- 识别结果到图像侧巡线覆盖/航向保持的映射

5. `Code/User/image/image_process.cc` 中的 `img_processing(...)` 负责：
- 斑马线停车链路
- 左右边线搜索
- 单边边线处理流水线
- 元素识别与状态机推进
- 中线融合
- 路径生成
- `pure_angle` 输出

补充说明：
- `Code/User/image/common.h` 是图像侧统一配置入口，图像算法阈值、默认开关和运行时默认值应优先集中在这里维护
- `Code/User/image/image_switch_utils.h` 提供图传链和识别链共用的布尔开关解析工具
- `Code/User/image/headfile.h` 与 `Code/User/image/image_headfile.h` 当前仅保留为兼容聚合头，新核心实现文件应优先按最小必要 include 引入依赖

最后，控制层在 `Code/User/src/PID.cc` 中读取全局 `pure_angle`，写入 `PID.vision_yaw`，再进入转向 PID。

完整数据流：

`camera.capture_frame(img)`
-> `640x480 原始图`
-> `resize(160x120) -> Gray -> OTSU -> 开闭运算 -> 边框清零`
-> `img_processing(binimg)`
-> `pts_left / pts_right / element_type / midline / pure_angle`
-> `PID.vision_yaw = pure_angle`
-> `PID.speed_adjust`
-> 左右轮目标速度与 PWM

## 2. 主入口与执行关系

### 2.1 系统入口

`main()` 中：
- 加载驱动模块
- `system_init()`
- 启动 PID、传感器、通信线程
- 终端设为非阻塞
- 通过 `StreamChain::ParseSwitch(...)` 和 `RecognitionChain::ParseSwitch(...)` 解析开关
- 调用 `Vision_System_Run()` 进入阻塞式视觉循环

### 2.2 视觉主循环

`Vision_System_Run()` 中的职责：

1. 初始化 `StreamChain`
   - 支持运行时开关：
   - `--stream`
   - `--no-stream`
   - `--stream=on`
   - `--stream=off`
2. 初始化识别覆盖状态 `recognition_follow_override = NONE`
3. 初始化 `RecognitionChain`
- `./model/cls.onnx`
- `./model/class_names.json`
   - 支持运行时开关：
   - `--recognition`
   - `--no-recognition`
   - `--recognition=on`
   - `--recognition=off`
4. 识别链内部维护识别模式：
- `RecognitionChain::Mode::NORMAL`
- `RecognitionChain::Mode::RECOGNITION`
5. 持续循环：
- 读取键盘 `c`
- 固定采集 `640x480`
- 巡线分支缩放到 `160x120`
- 根据当前模式进入普通巡线态或识别态
- 调用 `StreamChain::PublishFrame(view)` 更新图传画面

## 3. 关键全局状态量

这些状态量主要定义在 `image_data.h` / `image_data.cc`。

当前版本中，`image_data.*` 已不再只是“全局变量仓库”，还开始承载少量低风险的状态归口辅助函数，例如：
- `image_reset_far_line_state()`
- `image_reset_midline_path_state()`
- `image_reset_tracking_observation_state()`

这些函数的目的不是隐藏数据，而是先统一复位语义，减少跨文件重复写同一组全局状态。

### 3.1 边线与路径

- `pts_left`, `pts_right`
  - 左右边线完整处理上下文
  - 包含原始点、逆透视点、滤波点、重采样点、曲率、角点、中线

- `pts_far_left`, `pts_far_right`
  - 十字远端线处理上下文
  - 只在十字阶段找远端补线时使用

- `if_find_far_line`
  - 是否找到十字远端线

- `midline.mid`
  - 融合后的最终中线

- `midline.path`
  - 从 core 点并轨到中线后的控制路径

### 3.2 图像状态机

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
  - `CIRCLE_DIR_NONE`
  - `CIRCLE_DIR_LEFT`
  - `CIRCLE_DIR_RIGHT`

- `crossing_state`
  - `CROSSING_NONE`
  - `CROSSING_IN`
  - `CROSSING_RUNNING`

- `follow_mode`
  - `MIXED`
  - `MIDLEFT`
  - `MIDRIGHT`

### 3.3 模型识别相关

- `recognition_follow_override`
  - `NONE`
  - `FORCE_LEFT`
  - `FORCE_RIGHT`

识别链文件开头还有两个编译期开关：
- `BW_ENABLE_RECOGNITION`
  - 控制整条识别链是否启用
- `BW_ENABLE_RECOGNITION_ACTION`
  - 控制识别后是否执行切线/航向保持策略
  - 关闭后仍会识别、投票、打印结果，但不会输出 `FORCE_LEFT/FORCE_RIGHT`，也不会触发 `vehicle hold`

`RecognitionChain` 内部识别状态：
- `enabled_`
- `net_`
- `class_names_`
- `votes_`
- `last_result_`
- `last_label_`
- `required_votes_`
- `mode_`
  - `NORMAL`
  - `RECOGNITION`
- `recognition_timeout_ms_`
- `trigger_cooldown_until_ms_`
- `vehicle_hold_until_ms_`
- `vehicle_hold_yaw_`

代码中的识别全链路使用统一特殊注释标记：
- `[Recognition Chain Interface]`
- `[Recognition Chain Step 1]` 到 `[Recognition Chain Step 9]`

顺着这些标记即可从“模型加载”一路跟到“最终 pure_angle 输出”。

### 3.4 控制输出

- `pure_angle`
  - 图像侧最终输出的纯跟踪角
  - 右转为负，左转为正

- `circle_average_angle`
  - 环岛 RUNNING 阶段平均角
  - 当前版本已保留变量，但保护分支未启用

- `zebra_stop`
  - 视觉侧停车请求
  - 控制层处理真正停车

## 4. 普通巡线态完整链路

### 4.1 图像预处理

普通态下，`Vision_System_Run()` 对每帧做：

1. 保留原始采集图 `img`，分辨率固定为 `640x480`
2. `resize(img, line_img, 160x120, INTER_AREA)`
3. `cvtColor(line_img, binimg, COLOR_BGR2GRAY)`
4. `threshold(..., THRESH_BINARY | THRESH_OTSU)`
5. `morphologyEx(..., MORPH_OPEN)`
6. `morphologyEx(..., MORPH_CLOSE)`
7. 清零边界四边像素
8. 调用 `img_processing(binimg)`

### 4.2 `img_processing(...)` 入口阶段

当前版本中，`img_processing(...)` 已整理为“阶段函数编排”，主入口只负责组织顺序。主要阶段为：

1. `handle_zebra_stop_lifecycle(...)`
- 处理 `zebra_stop` 解锁、锁停和延迟停车生命周期
- 在锁停或 pending 阶段直接短路主链

2. `process_track_edges(...)`
- 完成左右边线搜索
- 调用 `process_line(...)` 跑单侧边线流水线

3. `try_trigger_zebra_pending_stop(...)`
- 在正常直道条件下检测斑马线
- 命中后进入延迟停车阶段并短路本帧

4. `update_track_state_machine(...)`
- 推进元素检测、环岛状态机、十字状态机
- 或在识别覆盖时统一屏蔽元素状态机

5. `build_midline_from_current_state(...)`
- 根据识别覆盖、十字远端线和普通跟线模式选择最终中线来源

6. `build_path_and_measure_pure_angle(...)`
- 从最终中线生成控制路径并计算测量角

7. `finalize_pure_angle_output(...)`
- 应用环岛平均角保护与丢线补偿
- 最后再叠加趋势前馈补偿

在斑马线相关阶段里，具体行为仍保持原语义：

1. 若检测到 `zebra_stop` 从 true 变 false
- 进入 cooldown
- 清空视觉缓存

2. 若 `zebra_stop == true`
- 持续保持停车请求
- `track_force_reset()`
- 清空边线/中线/路径
- `pure_angle = 0`
- 直接返回

3. 若处于 `zebra_pending == true`
- 清空视觉缓存
- 延迟计时结束后置位 `zebra_stop = true`
- 直接返回

### 4.3 左右边线搜索

完成斑马线处理后：

1. `SearchLine_Lpt(...)`
2. `SearchLine_Rpt(...)`

搜索起点默认在 `SET_IMAGE_CORE_X / SET_IMAGE_CORE_Y` 附近。

### 4.4 单边边线处理流水线

`process_line(true, pts_left)` 和 `process_line(false, pts_right)` 对左右边线做同一套处理：

1. 逆透视 `InversePerspectiveTransform`
2. 无效点过滤 `FilterValidPoints`
3. 滤波 `GetLinesFilter`
4. 重采样 `GetLinesResample`
5. 曲率计算 `local_curvature_points`
6. 曲率 NMS `nms_curvature`
7. 单侧中线生成
   - 左线走 `GetMidLine_Left`
   - 右线走 `GetMidLine_Right`
8. 中线再次等距重采样
9. 角点检测 `get_corner`
10. 曲线判定 `is_curve`

每条边线最终会得到：
- 原始边线点
- 逆透视点
- 滤波点
- 重采样点
- 曲率/角点
- 单侧中线
- `is_straight`
- `is_curve`

## 5. 元素检测与状态机链路

### 5.1 `element_detect()` 的职责

`element_detect()` 不直接做复杂控制，而是做“元素候选判定 + 投票 + 锁定”。

判定依据：
- 双角点且靠近近端：倾向十字
- 单侧角点 + 对侧局部更像直线：倾向环岛

内部静态量：
- `crossing_vote`
- `circle_vote`
- `protect`
- `last`

状态优先级：
- `CROSSING` 高于 `CIRCLE`
- `CIRCLE` 高于 `NORMAL`

锁定规则：
- 十字状态机在推进时，不允许环岛或 normal 覆盖
- 环岛状态机在推进时，不允许 normal 覆盖

### 5.2 手动复位

`track_force_reset()` 会立即执行：
- `element_type = NORMAL`
- `circle_state = CIRCLE_NONE`
- `crossing_state = CROSSING_NONE`
- `circle_direction = NONE`
- `follow_mode = MIXED`
- `if_find_far_line = false`
- 清空远端线缓存
- 调用 `roundabout_reset()`
- 调用 `crossing_reset()`

同时会请求 `element_detect()` 下一帧清空其内部投票和保护帧状态。

## 6. 环岛状态机

环岛状态机在 `roundabout_update()` 中推进，仅当 `element_type == CIRCLE` 时有效。

### 6.1 状态定义

#### `CIRCLE_BEGIN`
- 进入环岛前的贴外侧阶段
- 右环岛跟左中线
- 左环岛跟右中线
- 观察外侧边线丢失

#### `CIRCLE_IN`
- 切到内侧中线
- 观察对侧是否出现曲线
- 满足则转到 `CIRCLE_RUNNING`

#### `CIRCLE_RUNNING`
- 再次贴外侧更稳
- 观察对侧角点连续出现
- 满足则转到 `CIRCLE_OUT`

#### `CIRCLE_OUT`
- 切回内侧准备出环
- 观察对侧是否恢复直线
- 满足则转到 `CIRCLE_END`

#### `CIRCLE_END`
- 再回外侧跟线
- 通过“丢线 -> 找线”确认完全离开环岛
- 满足则退出到 `CIRCLE_NONE`

### 6.2 跟线模式切换

环岛状态机会动态修改 `follow_mode`：
- 右环岛：
  - `BEGIN` -> `MIDLEFT`
  - `IN` -> `MIDRIGHT`
  - `RUNNING` -> `MIDLEFT`
  - `OUT` -> `MIDRIGHT`
  - `END` -> `MIDLEFT`
- 左环岛相反

## 7. 十字状态机

十字状态机在 `crossing_update()` 中推进。

### 7.1 `CROSSING_IN`
- 只要双边都丢线连续达到阈值
- 转入 `CROSSING_RUNNING`

### 7.2 `CROSSING_RUNNING`
- 当左右边线重新都找回
- 退出为 `CROSSING_NONE`

### 7.3 十字远端补线

十字阶段还会调用 `crossing_far_line_check(img)`：

1. 清空远端线缓存
2. 优先用双角点连线中点向远处试探
3. 找到远端线起点后，分别爬远端左/右边线
4. 对远端线也跑 `process_line`
5. 得到 `pts_far_left.mid` 和 `pts_far_right.mid`
6. 若两侧都有有效远端中线，则置位 `if_find_far_line = true`

## 8. 识别覆盖链路

### 8.1 识别触发

在 `Vision_System_Run()` 的普通态中，`vision_runtime.cc` 通过 `recognition.TryEnterRecognition(...)` 调用识别链接口：

触发条件同时满足时进入识别态：
- ONNX 模型已成功加载
- 当前时间已过 cooldown
- `detect_red_rect_like(...)` 检出红色矩形目标

进入识别态时：
- `RecognitionChain::mode_ = RECOGNITION`
- 清空 `votes_`
- 设定 `recognition_timeout_ms_ = now + 2500ms`
- `track_force_reset()`
- `recognition_follow_override = NONE`
- 相机采集格式保持不变，继续使用固定 `640x480`

### 8.2 识别态内部流程

识别态中：

1. 不再调用普通 `img_processing`
2. 仅查找红色矩形 ROI
3. 将 ROI resize 到 `64x64`
4. 用 `cv::dnn::blobFromImage` 组输入
5. `net.forward()`
6. `minMaxLoc` 取类别索引
7. 写入 `votes_`

退出条件：
- `votes.size() >= required_votes`
- 或者超时

### 8.3 投票结果到车行为的映射

识别结束后：

- `weapon`
  - `last_result_ = WEAPON`
  - 普通态下设置 `recognition_follow_override = FORCE_LEFT`

- `supply`
  - `last_result_ = SUPPLY`
  - 普通态下设置 `recognition_follow_override = FORCE_RIGHT`

- `vehicle`
  - 不强制左右巡线
  - 记录识别前最后一份 `pure_angle`
  - `vehicle_hold_until_ms_ = now + 1000ms`
  - 后续 1 秒内主循环直接输出该 `pure_angle`

若 `BW_ENABLE_RECOGNITION_ACTION == 0`：
- 上述三类动作策略全部不执行
- 识别链仅保留“触发、识别、投票、日志输出”

- `unknown`
  - 不做巡线覆盖

退出识别态时：
- `track_force_reset()`
- `mode_ = NORMAL`
- `trigger_cooldown_until_ms_ = now + 3000ms`
- 相机采集格式保持不变，继续使用固定 `640x480`

## 9. 识别结果如何影响图像链路

`img_processing()` 中：

先判断：
- `recognition_override_active = (recognition_follow_override != NONE)`

### 9.1 无识别覆盖时

正常执行：
- `element_detect()`
- `roundabout_update()`
- `crossing_update()`
- `crossing_far_line_check()`
- `MID(...)`

### 9.2 有识别覆盖时

会做两件事：

1. 屏蔽元素状态机
- `element_type = NORMAL`
- `circle_state = CIRCLE_NONE`
- `crossing_state = CROSSING_NONE`
- `circle_direction = NONE`
- 清空远端线

2. 强制中线来源
- `FORCE_LEFT`：直接把 `pts_left.pts_resample` 复制为 `midline.mid`
- `FORCE_RIGHT`：直接把 `pts_right.pts_resample` 复制为 `midline.mid`

也就是说，模型识别并不是直接输出转角，而是改写“中线来源”。

## 10. 中线融合与路径生成

### 10.1 `MID(...)`

`MID(...)` 的作用是根据左右候选中线和 `follow_mode` 生成最终中线：

- `MIXED`
  - 两侧都存在时取平均
  - 一侧缺失时直接退回另一侧

- `MIDLEFT`
  - 优先取左候选中线
  - 在 `NORMAL` 状态可退回右候选中线

- `MIDRIGHT`
  - 优先取右候选中线
  - 在 `NORMAL` 状态可退回左候选中线

函数还会做：
- 截头
- 两侧起点对齐
- 小重合段保护

### 10.2 `BuildPathFromCoreToMidlineArc(...)`

该函数从车体 core 点出发生成控制路径：

1. 先选择中线上一个“前视并轨目标点”
2. 在 `core -> target_idx` 这段做渐进融合
3. `target_idx` 之后直接贴中线
4. 最后再整体等距重采样

输出是 `midline.path`。

## 11. `pure_angle` 输出

`CalculatePureAngleFromPath(...)`：

1. 先把车轴位置作为控制参考点
2. 默认取 `PUREANGLE_PREVIEW_BASE_IMAGE_Y` 对应的预瞄目标
3. 用 `MidLineSuggestPureAnglePreviewImageY(...)` 根据中线整条曲率链动态前推预瞄点
4. 曲率链为：整条 `mid` -> `local_curvature_points(...)` -> `nms_curvature(...)` -> 只平均 NMS 非零峰值
5. 预瞄前推阈值直接工作在曲率域 `1-cos(theta)`，不再做错误的角度换算
6. 当前未叠加 `S` 弯/直角弯/环岛的粗粒度最小 shift 覆盖，只由平均曲率决定基础 `shift`
7. 对动态预瞄图像行做“限速 + 轻滤波”过渡，避免预瞄点帧间跳变
8. 在 `midline.path` 上寻找最接近该预瞄位置的目标点
9. 用 `car -> target` 的割线方向计算 `atan2f(dx, forward)`
10. 限幅到 `[-80, 80]`

结果写入全局 `pure_angle`。

## 12. 控制层使用方式

控制线程中：

- `PID.vision_yaw = pure_angle`
- `PID.current_angle = PID.vision_yaw`
- 用 `PID_Angle` 计算 `PID.speed_adjust`
- 进一步生成左右轮速度目标与 PWM

因此图像侧对控制层的唯一主输出就是：
- `pure_angle`

停车链路额外输出：
- `zebra_stop`

## 13. 状态切换总表

### 13.1 顶层视觉模式

`NORMAL -> RECOGNITION`
- 检测到红色矩形
- 模型可用
- cooldown 已结束

`RECOGNITION -> NORMAL`
- 投票数够
- 或识别超时

### 13.2 识别覆盖状态

`NONE -> FORCE_LEFT`
- 识别结果为 `weapon`

`NONE -> FORCE_RIGHT`
- 识别结果为 `supply`

`FORCE_LEFT/FORCE_RIGHT -> NONE`
- 识别结果变为 `unknown`
- 新识别结果为 `vehicle`
- 手动按 `c`
- 进入识别态

### 13.3 元素状态

`NORMAL -> CROSSING`
- 双角点且十字投票达到阈值

`NORMAL -> CIRCLE`
- 单角点 + 对侧直线窗口成立
- 环岛投票达到阈值

`CROSSING/CIRCLE -> NORMAL`
- 状态机结束且保护帧结束
- 或被 `track_force_reset()`
- 或被强制复位判直道

### 13.4 斑马线状态

普通帧
-> 检测到斑马线
-> `zebra_pending = true`
-> 延时结束
-> `zebra_stop = true`
-> 控制层停车
-> 手动 Start 清除 `zebra_stop`
-> 进入 cooldown

## 14. 手动复位行为

按 `c` 后当前版本会做：
- `track_force_reset()`
- 调用 `RecognitionChain::Reset()`
- 清空识别投票
- `last_result_ = UNKNOWN`
- `last_label_ = "unknown"`
- `mode_ = NORMAL`
- 清空识别超时与 vehicle hold
- 清空触发 cooldown
- `recognition_follow_override = NONE`

识别链已经分离到 `Code/User/image/recognition_chain.*`，图传链已经分离到 `Code/User/image/stream_chain.*`，而 `main.cpp` 本身不再持有这些图像链内部状态成员。

这保证“手动复位”是真正全链路复位，而不只是复位元素状态机。

## 15. 本次全链路检查已修正项

### 15.1 手动 `c` 复位未清空识别结果

问题：
- 旧逻辑只清了 `track_force_reset()` 和 `recognition_follow_override`
- 但 `recognition.last_result` 仍然保留
- 下一帧普通态会重新根据 `last_result` 恢复 `FORCE_LEFT/FORCE_RIGHT`

现已修正：
- 手动 `c` 会同时清空识别结果、投票、hold 状态和模式状态

### 15.2 路径并轨目标点被临时代码覆盖

问题：
- `BuildPathFromCoreToMidlineArc(...)` 内部原来有一行临时覆盖：
- 直接把 `target_idx` 改成 `mid_count - 1`
- 导致前面通过参考前视位置选出来的并轨点完全失效

现已修正：
- 删除该临时覆盖
- 路径并轨重新按参考前视点逻辑工作

## 16. 当前保留但未启用的逻辑

以下逻辑当前代码中“有实现/有变量”，但最终分支未启用：

- `pure_angle_apply_lost_strategy(...)`
  - 丢线趋势外推与回线软切换
  - 目前最终应用分支被注释掉

- `circle_average_angle` 环岛平均角保护
  - 统计仍保留
  - 但最终替换 `pure_angle` 的代码被注释掉

因此当前实际输出更接近：
- 直接使用当帧路径算出来的 `pure_angle`
- 不再额外做丢线补偿/环岛平均角接管

如果后续要恢复，需要先重新验证：
- 丢线时输出是否更稳
- 环岛出环是否更平滑
- 是否会引入恢复时跳变

## 17. 建议阅读顺序

第一次看代码，建议按下面顺序：

1. `Code/main.cpp`
2. `Code/User/image/vision_runtime.cc`
3. `Code/User/image/stream_chain.cc`
4. `Code/User/image/recognition_chain.cc`
5. `Code/User/image/image_process.cc`
6. `Code/User/image/image_data.h`
7. `Code/User/image/element.cc`
8. `Code/User/image/element/circle.cc`
9. `Code/User/image/element/crossing.cc`
10. `Code/User/image/image_handle.cc`
11. `Code/User/image/image_midline_process.cc`
12. `Code/User/src/PID.cc`

这样最容易从“主循环 -> 状态机 -> 路径 -> 控制输出”串起来。
