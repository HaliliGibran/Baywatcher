# 识别板图像与事件链说明

本文档描述当前双板方案下识别板的真实主流程，只以当前主入口和默认构建为准。

## 0. 最近开发记录

### 2026-04-08 识别板测试链与图传调试收口

- 已完成
  - 识别默认开关改为 `BW_ENABLE_RECOGNITION=1`，默认不再上电就被关掉。
  - 识别板相机帧率收口到 `Code/User/inc/common.h` 的 `BW_RECOG_CAMERA_FPS`，当前默认 `30fps`，不再散落在入口代码里。
  - 新增测试门控 `BW_RECOG_REQUIRE_MANUAL_START`，当前默认 `1`。
  - 测试门控打开时，上电后不会直接开始红块检测；按一次 `c` 才启动一轮“红块检测 -> ROI -> 分类 -> 事件发送”。
  - 一轮识别态结束后会自动解除 armed，下一轮仍需再次按 `c`。
  - 识别板图传现在会在普通态和识别态都叠加：
    - 红块搜索范围框
    - 红块候选框/四边形
    - ROI 四边形
    - 当前触发状态文字
    - 右上角 ROI 截图预览
  - 修正了红块搜索带与触发目标带不一致的问题。
  - 之前识别链搜索带仍是旧的 `y=80~280`
  - 但触发目标带已经改到 `center_y=320±40`
  - 现在搜索带、红色掩膜最大 y、白色参考行都统一收口到 `common.h`
  - 图传链改成了参数化：
    - `BW_STREAM_PUBLISH_INTERVAL_FRAMES`
    - `BW_STREAM_MAX_WIDTH`
    - `BW_STREAM_JPEG_QUALITY`
  - 当前默认图传策略已经改成“每帧推送 + 下采样到最大宽度 400 + JPEG 质量 60”，优先保证浏览器里的调试连续性。

- 解决的问题
  - 解决了默认识别关闭导致“红块放进区域也完全不开始识别”的直接问题。
  - 解决了测试阶段无法只用一次按键控制识别链启停的问题。
  - 解决了识别板图传里看不到搜索范围、ROI 和触发结果的问题。
  - 解决了旧触发几何参数残留，导致搜索区域与目标区域错位的问题。
  - 缓解了识别板长期满速空转和图传 JPEG 压力过大的问题。

- 仍未完全解决
  - 当前 ONNX 单帧推理时间仍在 `300~400ms` 量级时，识别态图传依然不会非常流畅。
  - 这时主瓶颈已经不是 MJPEG 推流，而是板端 DNN 推理本身。
  - 如果后面实测 CPU 仍长期 100%，下一步优先级应是：
    - 继续降低模型复杂度
    - 缩短投票帧数
    - 或把识别态推理频率再单独限速

- 本轮没有做的事
  - 没有改 ONNX 模型本身
  - 没有改类别语义映射
  - 没有做本地交叉编译验证
  - 没有做实机双板联调验证

### 2026-04-08 红框搜索带回退到 80~280

- 已完成
  - 按当前测试需求，把识别板红框搜索带重新改回 `80 <= y < 280`
  - 同步把红色掩膜最大 y 改回 `280`
  - 同步把白色横向范围参考行改回 `y=280`
  - 所有参数仍然统一放在 `Code/User/inc/common.h`

- 解决的问题
  - 让识别板重新回到旧的上半区红框搜索口径，便于和你当前测试摆放方式保持一致

- 仍需注意
  - 这次只是把搜索窗口回退了，没有改 ROI 提取、分类、投票和事件发送逻辑
  - 如果红块实际摆放位置不在 `80~280` 这一带，识别链仍然不会触发

当前识别板职责固定为：

- 固定彩色采集 `640x480@BW_RECOG_CAMERA_FPS`
- 与 `yolo/project_root/scripts/_roi_runtime_geometry.py` 对齐的红块 ROI 提取
- ROI 分类与多帧投票
- 稳定结果边沿转成一次性事件
- `UART1@115200` 发送事件
- 图传当前彩色调试画面

当前识别板不负责：

- 巡线二值化
- 元素状态机
- `pure_angle`
- `speed_adjust`
- 本地左绕/右绕/保持航向执行

这些职责已经全部留在运行板。

## 1. 总体链路

识别板顶层执行关系如下：

`main.cpp`
-> `system_init()`
-> 解析 `stream` 和 `recognition` 开关
-> 进入 `RunRecognitionBoard(stream_enabled, recognition_enabled)`
-> 普通态做红色触发判断
-> 识别态做 ROI 分类和投票
-> 稳定结果生成一次性事件
-> `comm.send_event(action, seq)`
-> 运行板接收并执行本地动作

当前识别板不发持续状态包，不发 idle 包，只发事件。

## 1.1 当前工程结构

识别板当前用户代码已经按常规工程结构整理为：

- `Code/User/inc`
  - 放识别板用户头文件
  - 例如 `recognition_chain.h`、`recognition_runtime.h`、`stream_chain.h`、`transform_table.h`、`common.h`
- `Code/User/src`
  - 放识别板用户源文件
  - 例如 `recognition_chain.cc`、`recognition_runtime.cc`、`stream_chain.cc`、`transform_table.cc`、`Communication.cc`

原来的 `Code/User/image` 已经退出主代码目录，不再承载识别板实际源码。

## 2. 入口文件与执行关系

### 2.1 主入口

主入口在 `Code/main.cpp`，负责：

- 初始化相机 `640x480@BW_RECOG_CAMERA_FPS`
- 初始化双板串口 `UART1@115200`
- 设置终端为非阻塞
- 解析图传开关
- 解析识别开关
- 进入 `RunRecognitionBoard(...)`

识别板当前运行时开关有两组：

- 图传
  - `--stream`
  - `--no-stream`
  - `--stream=on|off`
  - `--stream-mode on|off`
- 识别链
  - `--recognition`
  - `--no-recognition`
  - `--recognition=on|off`
  - `--recognition-mode on|off`

### 2.2 识别运行时入口

主循环入口在 `Code/User/inc/recognition_runtime.h` 和 `Code/User/src/recognition_runtime.cc`：

- `kRecognitionFrameWidth = 640`
- `kRecognitionFrameHeight = 480`
- `kRecognitionFrameFps = BW_RECOG_CAMERA_FPS`
- `void RunRecognitionBoard(bool stream_enabled, bool recognition_enabled_by_switch);`

每帧顺序固定为：

1. 读取非阻塞终端输入；测试门控开启时，按一次 `c` 才 armed 一轮检测
2. 固定采彩色图 `640x480`
3. 若识别链被关闭，则显示 disabled 画面
4. 若已在识别态，执行 `ProcessRecognitionFrame(...)`
5. 若测试门控开启且尚未 armed，则显示“等待按 c 启动”画面
6. 若还在普通态且已 armed，执行 `TryEnterRecognition(...)`
7. 尝试 `TryGetNextTxEvent(...)`
8. 若有待发事件则 `comm.send_event(...)`
9. 图传开启时发布 `view`

## 3. 文件 owner 一览

| 文件 | 当前 owner |
| --- | --- |
| `Code/main.cpp` | 主入口、板端初始化、开关解析 |
| `Code/User/inc/recognition_runtime.h` + `Code/User/src/recognition_runtime.cc` | 识别板主循环调度 |
| `Code/User/inc/recognition_chain.h` + `Code/User/src/recognition_chain.cc` | 红框触发、分类、投票、事件生成 |
| `Code/User/inc/roi_runtime_geometry.h` + `Code/User/src/roi_runtime_geometry.cc` | 与 yolo 同步的 ROI 提取、IPM 回投、ROI 质量过滤 |
| `Code/User/inc/stream_chain.h` + `Code/User/src/stream_chain.cc` | 图传开关与图传发布 |
| `Code/User/inc/common.h` | 识别板编译期参数与默认开关 |
| `Code/User/inc/transform_table.h` + `Code/User/src/transform_table.cc` | 逆透视查表 |
| `Code/User/inc/Communication.h` + `Code/User/src/Communication.cc` | 双板事件协议与串口发送 |

## 4. 参数和开关在哪

识别板主要参数集中在 `Code/User/inc/common.h`。

### 4.1 功能开关

- `BW_ENABLE_RECOGNITION`
  - 识别链编译期默认开关
- `BW_ENABLE_STREAM`
  - 图传编译期默认开关

注意当前默认值：

- `BW_ENABLE_RECOGNITION = 1`
- `BW_ENABLE_STREAM = 1`
- `BW_RECOG_REQUIRE_MANUAL_START = 1`

也就是说：

- 默认上电是“图传开，识别开”
- 但测试门控默认也打开，所以上电后仍需先按一次 `c`，才开始一轮红块检测和后续识别

### 4.2 双板通信参数

- `BW_BOARD_EVENT_REPEAT_FRAMES`
  - 同一个新事件重复发送多少帧
  - 当前默认 `8`

### 4.3 红框触发几何参数

- `BW_RECOG_ROI_METHOD`
  - `0 = direct_red_quad`
  - `1 = ipm_square_from_top_edge`
  - 当前默认 `1`
  - 与 `yolo` 当前训练流程保持一致
- `BW_RECOG_CAMERA_FPS`
  - 识别板相机帧率
  - 当前默认 `30`
- `BW_RECOG_REQUIRE_MANUAL_START`
  - 是否要求按一次 `c` 才启动一轮检测
  - 当前默认 `1`
- `BW_RECOG_TRIGGER_SEARCH_Y_MIN / BW_RECOG_TRIGGER_SEARCH_Y_MAX`
  - 红块搜索带上下边界
- `BW_RECOG_RED_MASK_MAX_Y`
  - 红色掩膜允许处理到的最大 y
- `BW_RECOG_WHITE_REFERENCE_ROW_Y`
  - 白色横向范围估计参考行
- `BW_STREAM_PUBLISH_INTERVAL_FRAMES`
  - 图传推送间隔
- `BW_STREAM_MAX_WIDTH`
  - 图传最大宽度
- `BW_STREAM_JPEG_QUALITY`
  - 图传 JPEG 质量

### 4.4 运行时开关

图传开关和识别开关都由 `StreamChain::ParseSwitch(...)` 和 `RecognitionChain::ParseSwitch(...)` 统一解析。

## 5. 板端关键状态量

识别板最关键的状态都在 `RecognitionChain` 内部维护。

### 5.1 识别运行状态

- `enabled_`
  - 当前识别链是否真的启用
  - 受命令行开关和模型文件是否存在共同影响
- `mode_`
  - `NORMAL`
  - `RECOGNITION`
- `recognition_timeout_ms_`
  - 识别态超时时间
- `trigger_cooldown_until_ms_`
  - 退出识别态后的触发冷却截止时刻

### 5.2 投票相关

- `votes_`
  - 当前识别态里累计的分类结果
- `required_votes_`
  - 收敛需要的票数
  - 当前构造函数默认值 `8`

### 5.3 事件边沿相关

- `event_armed_`
  - 是否允许下一次同类事件重新触发
- `next_event_seq_`
  - 下一个事件序号
- `pending_tx_action_`
  - 当前待发送事件
- `pending_tx_seq_`
  - 当前待发送事件的序号
- `pending_tx_repeat_remain_`
  - 当前事件剩余重复发送帧数

## 6. 通信协议与双板协同

双板协议定义在 `Code/User/inc/Communication.h`。

当前使用的是固定长度二进制定长帧：

| 字段 | 含义 |
| --- | --- |
| `header1` | `0x5A` |
| `header2` | `0xA5` |
| `version` | 协议版本，当前 `0x01` |
| `seq` | 事件序号 |
| `action` | `BoardActionEvent` |
| `crc8` | 对 `version/seq/action` 做 CRC8 |
| `tail` | `0xED` |

动作事件枚举：

- `BoardActionEvent::WEAPON`
- `BoardActionEvent::SUPPLY`
- `BoardActionEvent::VEHICLE`

业务映射固定为：

- `weapon -> WEAPON -> 运行板左绕`
- `supply -> SUPPLY -> 运行板右绕`
- `vehicle -> VEHICLE -> 运行板保持当前航向`

当前识别板只负责发事件，不负责解释运行板动作。

## 7. 图传链

图传链在 `Code/User/src/stream_chain.cc`，对外接口头在 `Code/User/inc/stream_chain.h`。

行为：

- 由 `BW_ENABLE_STREAM` 给默认值
- 支持运行时命令行覆盖
- 服务启动后按 `BW_STREAM_PUBLISH_INTERVAL_FRAMES` 决定推送频率
- 发布前会按 `BW_STREAM_MAX_WIDTH` 等比缩放
- JPEG 质量由 `BW_STREAM_JPEG_QUALITY` 控制
- 图传只用于调试，不参与识别结果本身的决策

当前识别板图传画面：

- 未 armed 时显示原图加“按 c 启动测试”提示
- 普通态 armed 后会显示红块搜索框、触发状态和 ROI 预览
- 识别态会显示 `pred`、`infer ms`、`votes`、`result`

## 8. 模型与类别文件

识别链初始化在 `RecognitionChain::Initialize(...)` 中。

默认查找路径：

- `./model/cls.onnx`
- `./model/class_names.json`

运行时路径解析顺序：

1. 先按当前配置路径直接查
2. 再尝试可执行文件目录
3. 再尝试当前工作目录

如果模型文件不存在：

- `enabled_ = false`
- 识别链不上线
- 板端仍可继续图传和空闲运行

如果类别文件打开失败：

- 会回退到内置类别名
  - `supply`
  - `vehicle`
  - `weapon`

## 9. 普通态触发链

普通态触发入口在 `RecognitionChain::TryEnterRecognition(...)`。

进入识别态之前必须同时满足：

1. 识别链已启用
2. 当前处于 `NORMAL`
3. 不在触发冷却时间内
4. 当前帧能成功提取出 `rotated_roi`
5. ROI 通过低信息过滤
6. `event_armed_ == true`

### 9.1 ROI 提取逻辑

当前 ROI 提取已经与 `yolo/project_root/scripts/_roi_runtime_geometry.py` 对齐，核心约束包括：

1. 运行时前处理只裁 `BW_RECOG_TRIGGER_SEARCH_Y_MIN <= y < BW_RECOG_TRIGGER_SEARCH_Y_MAX` 这条带，并且红色搜索也只在这个窗口内进行
2. 搜索前还会读取原图 `y=BW_RECOG_WHITE_REFERENCE_ROW_Y` 这一行上的白色跑道范围，只保留该白区横向范围
   - 白色阈值固定为 `S <= 60`、`V >= 150`
   - 若最长连续白段宽度小于 `120` 像素，则回退到原来的整带搜索
3. 直接在裁带上转 HSV，不做高斯模糊
4. 两段红色阈值 + 核心红色阈值
5. 不做开闭运算，保留原始红色连通关系
6. 候选红块面积至少 `500`
7. 红块四角点内部必须基本全红
8. 逆透视后必须近似横向长方形，宽高比阈值 `1.15`
9. `direct_red_quad` 直接用上长边向上推出正方形
10. `ipm_square_from_top_edge` 先在逆透视坐标系里推正方形，再回投到原图
11. ROI 统一透视到 `64x64`
12. 轻微条带误选会被 `strip_reject`
13. 低纹理 ROI 会被 `low_info_reject` / `ipm_shallow_low_info_reject`

当前识别板这次只同步了白跑道横向范围约束，没有同步测试侧实验用的 `5:12` 红色标识块模型校验。

## 10. 识别态链路

一旦进入识别态：

- `mode_ = RECOGNITION`
- `votes_.clear()`
- `recognition_timeout_ms_ = t_ms + 2500`

之后每帧由 `ProcessRecognitionFrame(...)` 处理。

识别态每帧逻辑：

1. 重新执行一遍 ROI 提取
2. 只有 `rotated_roi + 质量过滤通过` 的帧才参与投票
3. 把 `64x64` ROI 送入 ONNX
4. 取 Top-1 类别索引
5. 把类别索引压入 `votes_`
6. 在 `view` 上叠加 ROI 状态、预测类别、推理耗时、投票进度
7. 若票数达到 `required_votes_` 或识别超时，则执行投票收敛

## 11. 投票收敛与类别映射

投票收敛逻辑也在 `ProcessRecognitionFrame(...)` 中。

当前策略：

1. 统计 `votes_` 里每个类别出现次数
2. 取出现次数最多的类别
3. 把类别文本映射成工程内目标类别

文本到目标类别的映射规则：

- 名字里包含 `weapon`
  - 映射成 `WEAPON`
- 名字里包含 `supply`
  - 映射成 `SUPPLY`
- 名字里包含 `vehicle`
  - 映射成 `VEHICLE`
- 其他
  - `NONE`

一旦得到合法事件：

- `pending_tx_action_ = action`
- `pending_tx_seq_ = next_event_seq_++`
- `pending_tx_repeat_remain_ = BW_BOARD_EVENT_REPEAT_FRAMES`
- `event_armed_ = false`

然后识别态退出：

- `mode_ = NORMAL`
- `trigger_cooldown_until_ms_ = t_ms + 3000`

## 12. 事件发送策略

识别板不是每帧都发当前状态，而是只在“稳定识别结果边沿”产生事件。

当前策略：

1. `TryGetNextTxEvent(...)` 每次只取一帧待发事件
2. 同一个新事件重复发送固定帧数
3. 重复次数耗尽后清空 `pending_tx_action_`
4. 目标离场后，普通态 `TryEnterRecognition(...)` 在“当前找不到红框”时重新把 `event_armed_` 置回 `true`

这意味着：

- 只要目标还没离场，即使再次进入相同识别，也不会反复生成同类新事件
- 只有“目标离场 -> 再次触发 -> 再次识别稳定”才会生成下一次事件

## 13. transform_table 的作用

`Code/User/inc/transform_table.h` 和 `Code/User/src/transform_table.cc` 提供逆透视查表：

- `UndistInverseMapH[480][640]`
- `UndistInverseMapW[480][640]`

当前识别板触发几何全部按 `640x480` 原图坐标工作，所以这张表也必须与 `480x640` 对齐。

它当前只服务于触发侧的几何约束：

- 把红框四角映射到逆透视平面
- 判断“横向长方形”是否成立

它不参与巡线，因为识别板已经不承担巡线。

## 14. 手动测试启动逻辑和使用方式

### 14.1 使用方式

识别板程序运行后，终端输入：

- `c`
- `C`

即可 armed 一轮红块检测与识别链。

### 14.2 当前实际行为

`recognition_runtime.cc` 的 `HandleManualRecognitionStart(...)` 当前会：

- `recognition->Reset()`
- 把测试门控置为 armed

也就是：

- 清空累计概率
- 回到 `NORMAL`
- 清超时与冷却
- 清待发事件
- 把事件链重新回到一轮新的 armed 起点

当前按 `c` 不会做：

- 重新初始化相机
- 重新初始化串口
- 改变图传开关
- 永久保持 armed；一轮识别态结束后会再次回到等待下一次 `c`

## 15. 识别板与运行板的边界

当前边界必须保持清晰：

- 识别板
  - 只决定“识别到了什么”
  - 不决定“车该怎么控”
- 运行板
  - 只决定“如何执行这个动作”
  - 不负责再做彩色识别

因此后续维护时：

- 识别触发条件、投票、类别映射、事件发送
  - 改识别板
- 绕行动作、保持航向、`pure_angle`、控制闭环
  - 改运行板

## 16. 当前不是主流程的内容

识别板当前主流程只依赖：

- `main.*`
- `recognition_runtime.*`
- `recognition_chain.*`
- `stream_chain.*`
- `transform_table.*`
- `Communication.*`
- `common.h`

不要再按旧单板 `vision_runtime.*` 或旧巡线/控制残留去理解识别板主链。
