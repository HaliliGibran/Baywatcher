#ifndef _USER_COMMON_H_
#define _USER_COMMON_H_

// 识别板图像侧统一配置入口
// -----------------------------------------------------------------------------
// 使用建议：
// 1. 先调“相机/曝光”，再调“预筛与 ROI 周期”，最后再碰“识别决策阈值”。
// 2. 优先通过降低无目标空跑负载来提稳定性，不要一开始就放宽 ROI/分类阈值。
// 3. 宏默认值面向当前 640x480、20fps、双板串口状态流方案。
// 4. 若实车出现“空闲 CPU 高、触发慢、误触发、重复进识别”，优先看：
//    - BW_RECOG_CAMERA_FPS
//    - BW_RECOG_NORMAL_PRECHECK_INTERVAL_MS
//    - BW_RECOG_FULL_ROI_MIN_INTERVAL_MS
//    - BW_RECOG_LOOP_TARGET_FPS
//    - BW_RECOG_STATE_HEARTBEAT_INTERVAL_MS
//
// 调参优先级约定：
// - [先调]：优先用于解决“CPU/时延/图传/触发快慢”问题，风险相对低。
// - [谨慎调]：会明显影响状态判定、误触发/漏检平衡，需要带场景对比。
// - [一般别动]：属于几何/方法口径，除非确认当前假设不成立，否则不要先碰。

#pragma region A. 顶层功能开关与运行模式
// [先调] 识别链编译期默认开关
// 作用：
// - 决定程序默认是否允许进入识别链。
// - 仍可被运行时参数 --recognition / --no-recognition 覆盖。
// 调参建议：
// - 只作为总开关，正常实车一般保持 1。
#ifndef BW_ENABLE_RECOGNITION
#define BW_ENABLE_RECOGNITION 1
#endif

// [先调] 图传编译期默认开关
// 作用：
// - 决定程序默认是否启动 MJPEG 图传链。
// - 仍可被运行时参数 --stream / --no-stream 覆盖。
// 调参建议：
// - 查识别 ROI 时开，跑负载/时延时关。
#ifndef BW_ENABLE_STREAM
#define BW_ENABLE_STREAM 0
#endif

// [先调] 手动启动门控
// 作用：
// - 1：上电后先不跑红块检测，按一次 c 才启动一次完整识别链。
// - 0：识别链上电即持续运行。
// 典型用途：
// - 单板联调、静态拍照验证时设为 1。
// - 双板实车联调时通常设为 0。
#ifndef BW_RECOG_REQUIRE_MANUAL_START
#define BW_RECOG_REQUIRE_MANUAL_START 0
#endif
#pragma endregion

#pragma region B. 相机采集与曝光
// [先调] 相机请求帧率
// 作用：
// - 运行时主链向相机请求的目标 fps。
// - 真实 fps 以启动日志中的 actual_fps 为准。
// 调大效果：
// - 红块出现到进入识别的响应更快。
// - CPU、带宽、图传压力更高。
// 调小效果：
// - 空跑负载更低，图传更稳。
// - 红块触发延迟略增。
// 建议：
// - 先用 20 跑通，再考虑往上加。
#ifndef BW_RECOG_CAMERA_FPS
#define BW_RECOG_CAMERA_FPS 20
#endif

// [先调] 是否启用固定手动曝光
// 作用：
// - 0：沿用相机自动曝光。
// - 1：启动时调用手动曝光接口，压制自动曝光跳闪。
// 调参建议：
// - 识别不稳、画面时亮时暗时优先开 1。
// - 场地光照变化极大时才考虑回到 0。
#ifndef BW_RECOG_CAMERA_USE_MANUAL_EXPOSURE
#define BW_RECOG_CAMERA_USE_MANUAL_EXPOSURE 1 
#endif

// [先调] 手动曝光值
// 作用：
// - 仅在 BW_RECOG_CAMERA_USE_MANUAL_EXPOSURE=1 时生效。
// 调大效果：
// - 画面更亮，暗处更容易看清，但高光更容易发白。
// 调小效果：
// - 高光压得更稳，但暗部可能丢失红色细节。
// 建议：
// - 先固定场地，在 80~140 区间实车扫一轮。
#ifndef BW_RECOG_CAMERA_MANUAL_EXPOSURE
#define BW_RECOG_CAMERA_MANUAL_EXPOSURE 120
#endif

// [谨慎调] 是否优先尝试低 CPU MJPG 模式
// 作用：
// - 1：先尝试 LQ_CAMERA_0CPU_MJPG，失败再回退。
// - 0：直接用 LQ_CAMERA_HIGH_MJPG。
// 当前默认 0 的原因：
// - 当前实机摄像头已知不稳定兼容该低 CPU 模式。
// 建议：
// - 仅在确认摄像头兼容时再开 1。
#ifndef BW_RECOG_CAMERA_TRY_0CPU_MJPG
#define BW_RECOG_CAMERA_TRY_0CPU_MJPG 0
#endif

// [一般别动] ROI 提取方法
// 作用：
// - 当前文档重构后的主链固定使用 `ipm_square_from_top_edge`。
// - 这里保留宏仅为兼容旧配置，不再作为运行时主链切换入口。
// 0 = direct_red_quad
// 1 = ipm_square_from_top_edge
#ifndef BW_RECOG_ROI_METHOD
#define BW_RECOG_ROI_METHOD 1
#endif

#pragma endregion

#pragma region C. 主循环限频、性能统计与日志
// [先调] 是否启用主循环阶段耗时统计
// 作用：
// - 1：按 1s 窗口输出空闲无目标态 PERF 统计。
// - 0：完全关闭性能统计，减少一点点终端输出和字符串开销。
// 注意：
// - PERF 只统计“无红、无识别态”的空闲循环，不代表识别态最坏耗时。
#ifndef BW_RECOG_ENABLE_PERF_LOG
#define BW_RECOG_ENABLE_PERF_LOG 1
#endif

// [先调] 主循环目标 fps
// 作用：
// - >0：活动态按固定目标 fps 限频。
// - <=0：不做固定限频，保持“相机驱动 + sleep”原行为。
// 调大效果：
// - 响应更快，但 CPU、更高频率预筛/ROI/图传压力都会增大。
// 调小效果：
// - CPU 和链路更稳，但识别响应和图传刷新率下降。
// 建议：
// - 先用 20，确认逻辑稳定后再考虑往上试。
#ifndef BW_RECOG_LOOP_TARGET_FPS
#define BW_RECOG_LOOP_TARGET_FPS 20
#endif

// [先调] 动态频率：普通无目标态
// 作用：
// - NORMAL 且最近没有红色候选时，主循环按该频率运行。
// - 这是当前空跑 CPU 的主要调参入口。
#ifndef BW_RECOG_LOOP_FPS_NORMAL
#define BW_RECOG_LOOP_FPS_NORMAL 10
#endif

// [先调] 动态频率：候选态
// 作用：
// - 最近出现红色候选，或当前状态已进入 u/NO_RESULT 时使用。
// - 兼顾“尽快拉起检测”与“不要一直全速空跑”。
#ifndef BW_RECOG_LOOP_FPS_CANDIDATE
#define BW_RECOG_LOOP_FPS_CANDIDATE 25
#endif

// [先调] 动态频率：识别态
// 作用：
// - mode == RECOGNITION 时使用。
// - 目标是尽快完成当前 2 帧分类，不在识别态拖太久。
#ifndef BW_RECOG_LOOP_FPS_RECOGNITION
#define BW_RECOG_LOOP_FPS_RECOGNITION 30
#endif

// [先调] 动态频率：成功结果锁存保持态
// 作用：
// - 已成功识别出 v/w/s，只是在等待失标释放时使用。
// - 该阶段无需高频跑全搜索带，保持低频即可。
#ifndef BW_RECOG_LOOP_FPS_LATCHED
#define BW_RECOG_LOOP_FPS_LATCHED 10
#endif

// [先调] 识别链详细日志开关
// 作用：
// - 0：关闭逐帧/高频日志。
// - 1：开启调试日志。
// 关闭后仍保留关键状态变化日志：
// - entering recognition
// - final result
// - state_out
// - sign lost / release
// 建议：
// - 实车常态保持 0，排查具体 reject 原因时再开 1。
#ifndef BW_RECOG_VERBOSE_LOG
#define BW_RECOG_VERBOSE_LOG 0
#endif

// [谨慎调] 状态心跳周期（毫秒）
// 作用：
// - 状态变化时立即发包。
// - 状态未变化时，最多每隔该周期补发一次 heartbeat。
// 调大效果：
// - 串口负载更低，但运行板 stale timeout 容错更小。
// 调小效果：
// - 心跳更密，运行板更不易误判掉线，但串口更忙。
// 建议：
// - 一般不应大于运行板 stale timeout 的 1/3 ~ 1/2。
#ifndef BW_RECOG_STATE_HEARTBEAT_INTERVAL_MS
#define BW_RECOG_STATE_HEARTBEAT_INTERVAL_MS 50
#endif
#pragma endregion

#pragma region D. 主循环空闲/活动 sleep 策略
// [先调] 空闲态 sleep（毫秒）
// 作用：
// - 仅用于识别关闭或等待按 c 启动时，降低空转 CPU。
// 调大效果：
// - 空闲 CPU 更低。
// - 手动启动后第一帧响应略慢。
#ifndef BW_RECOG_IDLE_SLEEP_MS
#define BW_RECOG_IDLE_SLEEP_MS 8
#endif

// [先调] 活动态额外 sleep（毫秒）
// 作用：
// - 活动态下的额外让步。
// 当前默认 0 的原因：
// - 已引入固定 loop target fps，不再需要再额外 sleep。
// 建议：
// - 若已使用 BW_RECOG_LOOP_TARGET_FPS，通常保持 0。
#ifndef BW_RECOG_ACTIVE_SLEEP_MS
#define BW_RECOG_ACTIVE_SLEEP_MS 0
#endif
#pragma endregion

#pragma region E. 普通态预筛与 ROI 节流
// [先调] 普通态 HSV 预筛周期（毫秒）
// 作用：
// - 仅在 NORMAL 且未锁存成功结果时使用。
// - 0 表示每帧都做 HSV 预筛。
// 调大效果：
// - 空跑 CPU 更低。
// - 小红块首次出现到被发现的延迟增大。
// 调小效果：
// - 红块进入触发链更快。
// - 空跑负载更高。
// 当前默认 20ms：
// - 对应约 50Hz 预筛，兼顾响应和负载。
#ifndef BW_RECOG_NORMAL_PRECHECK_INTERVAL_MS
#define BW_RECOG_NORMAL_PRECHECK_INTERVAL_MS 20
#endif

// [谨慎调] 普通态 HSV 预筛最小红像素数
// 作用：
// - 低于该值时，不值得进入完整 ROI 提取。
// 调大效果：
// - 抑制小噪声和远处很小的红块，但可能推迟真正小标识板的进入。
// 调小效果：
// - 更敏感，但误触发和空跑 ROI 次数会上升。
#ifndef BW_RECOG_NORMAL_PRECHECK_MIN_PIXELS
#define BW_RECOG_NORMAL_PRECHECK_MIN_PIXELS 120
#endif

// [先调] 是否启用超轻量 BGR 采样预筛
// 作用：
// - 作为 HSV 预筛前的一层更便宜过滤。
// - 只要这层阴性，就直接跳过 HSV 预筛。
// 建议：
// - 绝大多数情况下保持 1。
// - 只有怀疑它误杀远处红块时才关掉做 AB 对比。
#ifndef BW_RECOG_ULTRA_FAST_PRECHECK_ENABLE
#define BW_RECOG_ULTRA_FAST_PRECHECK_ENABLE 1
#endif

// [谨慎调] 超轻量红采样步长（像素）
// 调大效果：
// - 更省 CPU，但更容易漏掉很小的红块。
// 调小效果：
// - 更敏感，但会增加预筛扫描开销。
// 建议：
// - 4 是当前折中值；若远处小红块漏检，可降到 2。
#ifndef BW_RECOG_ULTRA_RED_SAMPLE_STEP
#define BW_RECOG_ULTRA_RED_SAMPLE_STEP 4
#endif

// [谨慎调] 超轻量红采样命中最小样本数
// 调大效果：
// - 更不容易误触发 HSV 预筛。
// 调小效果：
// - 更容易进入 HSV 预筛，响应更快但空跑负载增加。
#ifndef BW_RECOG_ULTRA_RED_MIN_SAMPLES
#define BW_RECOG_ULTRA_RED_MIN_SAMPLES 8
#endif

// [先调] 普通态完整 ROI 提取最小周期（毫秒）
// 作用：
// - 即使预筛阳性，也不在 NORMAL 态每帧都做 ExtractRotatedRoi。
// 调大效果：
// - CPU 更低。
// - 红块进入触发链和切到 u/进入识别的延迟变大。
// 调小效果：
// - 触发更快。
// - NORMAL 态完整 ROI 开销更高。
// 建议：
// - 通常与 BW_RECOG_NORMAL_PRECHECK_INTERVAL_MS 取同一量级。
#ifndef BW_RECOG_FULL_ROI_MIN_INTERVAL_MS
#define BW_RECOG_FULL_ROI_MIN_INTERVAL_MS 20
#endif

// [一般别动] 识别态局部 ROI 跟踪搜索框扩张倍数
// 作用：
// - 进入 RECOGNITION 后，优先在上一帧 bbox 附近局部搜索。
// - 该值决定局部搜索框相对上一帧 bbox 的放大程度。
// 调大效果：
// - 局部搜索更稳，不容易丢目标。
// - 识别态每帧局部搜索开销变大，更接近全搜索。
// 调小效果：
// - 局部搜索更省，但 ROI 抖动/移动时更容易回退到全搜索。
#ifndef BW_RECOG_TRACK_ROI_EXPAND_RATIO
#define BW_RECOG_TRACK_ROI_EXPAND_RATIO 2.0f
#endif

// [谨慎调] 识别态最少有效推理帧数
// 作用：
// - 达到该帧数后，若概率和 margin 达标，就可以提前给结果。
// 调大效果：
// - 结果更稳，但输出更慢。
// 调小效果：
// - 结果更快，但更容易抖。
#ifndef BW_RECOG_MIN_VALID_FRAMES
#define BW_RECOG_MIN_VALID_FRAMES 2
#endif

// [谨慎调] 识别态最多累计推理帧数
// 作用：
// - 到达该帧数后，无论是否提前收敛，都必须收敛出一次结果或失败结论。
// 建议：
// - 通常与 MIN_VALID_FRAMES 配套调。
// - 若设得更大，必须同时关注识别态停留时间和重复进入的风险。
#ifndef BW_RECOG_MAX_VALID_FRAMES
#define BW_RECOG_MAX_VALID_FRAMES 2
#endif
#pragma endregion

#pragma region F. 图传链参数
// [先调] 图传发布帧间隔
// 作用：
// - 1 表示每帧都发。
// - 2 表示隔一帧发。
// 调大效果：
// - 图传更省 CPU/带宽。
// - 页面 fps 和实时感下降。
#ifndef BW_STREAM_PUBLISH_INTERVAL_FRAMES
#define BW_STREAM_PUBLISH_INTERVAL_FRAMES 1
#endif

// [先调] 图传最大宽度
// 作用：
// - >0 时，发布前按比例缩放，减轻 JPEG 编码和网络压力。
// 调大效果：
// - 图更清晰，但编码和网络负载更高。
// 调小效果：
// - 图更糊，但延迟和帧率更稳。
#ifndef BW_STREAM_MAX_WIDTH
#define BW_STREAM_MAX_WIDTH 400
#endif

// [先调] 图传 JPEG 质量
// 作用：
// - 控制 MJPEG 编码体积与清晰度。
// 调大效果：
// - 画质更好，延时/带宽更差。
// 调小效果：
// - 图传更顺，但细节损失更多。
#ifndef BW_STREAM_JPEG_QUALITY
#define BW_STREAM_JPEG_QUALITY 60
#endif

// [谨慎调] 图传输出裁剪下边界（开区间）
// 作用：
// - 仅影响发布画面，不影响内部 640x480 原图、ROI 提取和分类。
// 调大效果：
// - 图传看得更多，但编码开销更高。
// 调小效果：
// - 图传更省，但调试可视信息减少。
#ifndef BW_STREAM_CROP_MAX_Y
#define BW_STREAM_CROP_MAX_Y 280
#endif

// [谨慎调] 图传上叠加 ROI 预览边长（像素）
// 调大效果：
// - ROI 看得更清楚，但遮挡原图更多。
// 调小效果：
// - 更省空间，但不利于观察 ROI 内容。
#ifndef BW_RECOG_ROI_PREVIEW_SIZE
#define BW_RECOG_ROI_PREVIEW_SIZE 112
#endif
#pragma endregion

#pragma region G. 双板通信与链路容错
// [先调] 串口首次初始化失败后的额外重试次数
// 作用：
// - 首次失败后，再按固定节奏尝试若干次。
// 调大效果：
// - 更不容易因上电时序错过串口。
// - 启动阶段等待更久。
#ifndef BW_BOARD_COMM_INIT_RETRY_TIMES
#define BW_BOARD_COMM_INIT_RETRY_TIMES 10
#endif

// [先调] 串口初始化重试间隔（毫秒）
// 调大效果：
// - 重试更温和，日志更少。
// 调小效果：
// - 更快探测设备恢复，但启动期更密集重试。
#ifndef BW_BOARD_COMM_INIT_RETRY_INTERVAL_MS
#define BW_BOARD_COMM_INIT_RETRY_INTERVAL_MS 1000
#endif
#pragma endregion

#pragma region H. 红块面积阈值
// [谨慎调] 非噪声红块最小面积
// 作用：
// - 低于该值直接当噪声。
// 调大效果：
// - 抗噪更强，但远处小红块更容易被忽略。
// 调小效果：
// - 更敏感，但普通噪声更容易进入链路。
#ifndef BW_RECOG_RED_MIN_AREA
#define BW_RECOG_RED_MIN_AREA 500
#endif

// [谨慎调] 松弛红块候选最小面积
// 作用：
// - 只用于“是否值得尝试标识板/IPM 路径”这一层。
// - 不改变砖块阈值与正式 ROI 质量门。
// 调大效果：
// - 减少很小红块带来的无效 loose IPM。
// 调小效果：
// - 小红块更容易提前进入 u / sign-like 路径。
#ifndef BW_RECOG_LOOSE_RED_MIN_AREA
#define BW_RECOG_LOOSE_RED_MIN_AREA 200
#endif

// [谨慎调] 砖块面积阈值
// 作用：
// - 在“不满足识别标识”的前提下，面积达到该阈值则回传 b，否则回传 n。
// 调大效果：
// - 更少把大红块判成砖块。
// 调小效果：
// - 更容易进入 b，可能压掉靠近底部的大标识板。
#ifndef BW_RECOG_BRICK_MIN_AREA
#define BW_RECOG_BRICK_MIN_AREA 2000
#endif
#pragma endregion

#pragma region I. 红块搜索带与几何约束
// [一般别动] 红块搜索带上边界
// 作用：
// - 与《红带分类与ROI提取流程.md》保持一致。
// - 当前固定流程要求基础搜索带为 y=160..320。
#ifndef BW_RECOG_TRIGGER_SEARCH_Y_MIN
#define BW_RECOG_TRIGGER_SEARCH_Y_MIN 160
#endif

// [一般别动] 红块搜索带下边界（开区间）
// 作用：
// - 与《红带分类与ROI提取流程.md》保持一致。
// - 当前固定流程要求基础搜索带为 y=160..320。
#ifndef BW_RECOG_TRIGGER_SEARCH_Y_MAX
#define BW_RECOG_TRIGGER_SEARCH_Y_MAX 320
#endif

// [一般别动] 红色掩膜允许处理到的最大 y（开区间）
// 作用：
// - 与《红带分类与ROI提取流程.md》保持一致。
// - 严格红带掩码默认处理到 y=320。
#ifndef BW_RECOG_RED_MASK_MAX_Y
#define BW_RECOG_RED_MASK_MAX_Y 320
#endif

// [一般别动] 白色参考行
// 作用：
// - 与《红带分类与ROI提取流程.md》保持一致。
// - 当前固定流程要求 white_reference_row_y = 320。
#ifndef BW_RECOG_WHITE_REFERENCE_ROW_Y
#define BW_RECOG_WHITE_REFERENCE_ROW_Y 320
#endif

// [一般别动] 逆透视横向长方形约束总开关
// 作用：
// - 1：要求候选红块在 IPM 中更像横向长方形。
// - 0：关闭这层几何约束。
// 建议：
// - 只有在确认这层几何过严误杀标识板时，才临时关掉做对比。
#ifndef BW_RECOG_TRIGGER_IPM_RECT_ENABLE
#define BW_RECOG_TRIGGER_IPM_RECT_ENABLE 1
#endif

// [一般别动] 逆透视横向长方形最小宽高比
// 调大效果：
// - 更严格要求“横向长条”，误把竖向噪声当标识板的概率更低。
// - 也更容易拒绝透视畸变较大的真实标识板。
// 调小效果：
// - 更容易通过 loose IPM，但误触发更多。
#ifndef BW_RECOG_TRIGGER_IPM_MIN_WIDTH_HEIGHT_RATIO
#define BW_RECOG_TRIGGER_IPM_MIN_WIDTH_HEIGHT_RATIO 1.30f
#endif

#pragma endregion

#pragma region J. 识别结果保持策略
// [谨慎调] 成功识别后的失标保持时间（毫秒）
// 作用：
// - 已得到 v/w/s 后，短暂丢目标时继续保持上次成功结果。
// 调大效果：
// - 更不容易重复进识别。
// - 错把另一块新标识板延后识别的风险也更高。
// 调小效果：
// - 更快回到 n，但更容易因几何抖动反复重进识别。
#ifndef BW_RECOG_SIGN_LOSS_HOLD_MS
#define BW_RECOG_SIGN_LOSS_HOLD_MS 1500
#endif
#pragma endregion

#endif
