# 识别板图像与事件链说明

本文档描述当前双板方案下识别板的真实主流程，只以当前主入口和默认构建为准。

当前识别板职责固定为：

- 固定彩色采集 `640x480@180`
- 红框触发检测
- 红框 ROI 分类与多帧投票
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

- 初始化相机 `640x480@180`
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
- `kRecognitionFrameFps = 180`
- `void RunRecognitionBoard(bool stream_enabled, bool recognition_enabled_by_switch);`

每帧顺序固定为：

1. 手动复位识别链
2. 固定采彩色图 `640x480`
3. 若已在识别态，执行 `ProcessRecognitionFrame(...)`
4. 若还在普通态，执行 `TryEnterRecognition(...)`
5. 若本帧未进入识别态，则显示 idle 画面
6. 尝试 `TryGetNextTxEvent(...)`
7. 若有待发事件则 `comm.send_event(...)`
8. 图传开启时发布 `view`

## 3. 文件 owner 一览

| 文件 | 当前 owner |
| --- | --- |
| `Code/main.cpp` | 主入口、板端初始化、开关解析 |
| `Code/User/inc/recognition_runtime.h` + `Code/User/src/recognition_runtime.cc` | 识别板主循环调度 |
| `Code/User/inc/recognition_chain.h` + `Code/User/src/recognition_chain.cc` | 红框触发、分类、投票、事件生成 |
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

- `BW_ENABLE_RECOGNITION = 0`
- `BW_ENABLE_STREAM = 1`

也就是说：

- 默认上电是“图传开，识别关”
- 若要实际识别，需要命令行显式开识别，或把宏改为 1

### 4.2 双板通信参数

- `BW_BOARD_EVENT_REPEAT_FRAMES`
  - 同一个新事件重复发送多少帧
  - 当前默认 `8`

### 4.3 红框触发几何参数

- `BW_RECOG_TRIGGER_CENTER_Y`
  - 触发带中心行
  - 当前坐标基于 `640x480` 原图
  - 当前默认 `320`
- `BW_RECOG_TRIGGER_CENTER_Y_TOL`
  - 触发带上下容差
  - 当前默认 `40`
- `BW_RECOG_TRIGGER_IPM_RECT_ENABLE`
  - 是否启用逆透视横向长方形判定
- `BW_RECOG_TRIGGER_IPM_MIN_WIDTH_HEIGHT_RATIO`
  - 逆透视横向长方形最小宽高比
  - 当前默认 `1.30`

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
- 服务启动后每两帧发布一次
- 图传只用于调试，不参与识别结果本身的决策

当前识别板图传画面：

- 普通态显示原图加 idle 提示
- 命中红框时会画出触发框
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
4. 原图中检测到红色近矩形目标
5. 红框中心 `y` 落在触发带
6. 若开启了 IPM 判定，逆透视后必须近似横向长方形
7. `event_armed_ == true`

### 9.1 红框检测逻辑

红框检测函数是 `detect_red_rect_like(...)`。

当前做法：

1. 原图 BGR 转 HSV
2. 两段红色阈值合并
3. 开运算 + 闭运算
4. 找外轮廓
5. 过滤掉太小、不够像矩形、填充率太低的候选
6. 用面积、填充率、宽高比综合打分，取最优矩形

主要硬规则：

- 面积至少 `300`
- 宽高至少 `12`
- 填充率至少 `0.55`

### 9.2 触发带逻辑

红框中心行满足：

- `center_y` 在 `[BW_RECOG_TRIGGER_CENTER_Y - BW_RECOG_TRIGGER_CENTER_Y_TOL, BW_RECOG_TRIGGER_CENTER_Y + BW_RECOG_TRIGGER_CENTER_Y_TOL]`

按当前默认值，就是：

- `[280, 360]`

### 9.3 逆透视横向长方形判定

如果 `BW_RECOG_TRIGGER_IPM_RECT_ENABLE == 1`：

1. 用 `transform_table` 把红框四个角映射到逆透视平面
2. 计算上下边长度和左右边长度
3. 要求：
   - 上边明显长于左右边
   - 下边明显长于左右边
   - 最小宽高比满足 `BW_RECOG_TRIGGER_IPM_MIN_WIDTH_HEIGHT_RATIO`

这一步的目的不是分类，而是拒绝掉几何上明显不符合目标板朝向的红框。

## 10. 识别态链路

一旦进入识别态：

- `mode_ = RECOGNITION`
- `votes_.clear()`
- `recognition_timeout_ms_ = t_ms + 2500`

之后每帧由 `ProcessRecognitionFrame(...)` 处理。

识别态每帧逻辑：

1. 再次找红框
2. 若找到红框
   - 截出 ROI
   - resize 到 `64x64`
   - `blobFromImage`
   - ONNX 推理
   - 取 Top-1 类别索引
   - 把类别索引压入 `votes_`
3. 在 `view` 上叠加预测类别、推理耗时、投票进度
4. 若票数达到 `required_votes_` 或识别超时，则执行投票收敛

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

## 14. 手动复位逻辑和使用方式

### 14.1 使用方式

识别板程序运行后，终端输入：

- `c`
- `C`

即可触发手动复位。

### 14.2 当前实际复位内容

`recognition_runtime.cc` 的 `HandleManualRecognitionReset(...)` 当前只会：

- `recognition->Reset()`

也就是：

- 清空 `votes_`
- 回到 `NORMAL`
- 清超时与冷却
- 清待发事件
- 把 `event_armed_` 恢复为允许再次触发
- 把 `next_event_seq_` 重新置回 `0`

当前手动复位不会做：

- 重新初始化相机
- 重新初始化串口
- 改变图传开关

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
