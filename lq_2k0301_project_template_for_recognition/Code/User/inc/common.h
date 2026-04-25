#ifndef _USER_COMMON_H_
#define _USER_COMMON_H_

// 识别板图像侧统一配置入口：
// - 当前只保留识别链和图传链真实使用的编译期参数。
// - 旧单板巡线/元素/控制参数已从识别板工程中剥离。

#pragma region 识别板功能开关
// 识别链编译期默认开关：
// 运行时仍可通过 --recognition / --no-recognition 覆盖。
#ifndef BW_ENABLE_RECOGNITION
#define BW_ENABLE_RECOGNITION 1
#endif

// 图传编译期默认开关：
// 运行时仍可通过 --stream / --no-stream 覆盖。
#ifndef BW_ENABLE_STREAM
#define BW_ENABLE_STREAM 1
#endif

// 测试用手动启动门控：
// 1 表示上电后先不跑红块检测，按一次 c 后才开始“红块检测 -> ROI -> 分类 -> 发事件”整条链。
// 0 表示识别链上电即按正常逻辑持续运行。
#ifndef BW_RECOG_REQUIRE_MANUAL_START
#define BW_RECOG_REQUIRE_MANUAL_START 0
#endif

// 识别板相机输出帧率：
// recognition_runtime.h/main.cpp 等运行时入口统一从这里取值，避免再散落在源码里。
#ifndef BW_RECOG_CAMERA_FPS
#define BW_RECOG_CAMERA_FPS 30
#endif

// 成功识别后的失标保持时间（毫秒）：
// 当已经得到 v/w/s 后，只要标识板短暂离开视野，仍保持最后结果一段时间；
// 若在该时间内重新看到标识板，则继续保持，不重复进识别。
#ifndef BW_RECOG_SIGN_LOSS_HOLD_MS
#define BW_RECOG_SIGN_LOSS_HOLD_MS 3000
#endif

// 识别板是否启用固定手动曝光：
// - 0：沿用摄像头默认自动曝光
// - 1：上电后调用 lq_camera_ex::set_exposure_manual(...) 固定曝光，优先用于抑制自动曝光跳闪
// 当前新库公开接口只有手动曝光，没有“自动曝光参数调优”入口。
#ifndef BW_RECOG_CAMERA_USE_MANUAL_EXPOSURE
#define BW_RECOG_CAMERA_USE_MANUAL_EXPOSURE 1 
#endif

// 识别板手动曝光值：
// 仅在 BW_RECOG_CAMERA_USE_MANUAL_EXPOSURE=1 时生效。
// 新库头文件建议值大致在 [20, 200]，实车需结合场地亮度再调。
#ifndef BW_RECOG_CAMERA_MANUAL_EXPOSURE
#define BW_RECOG_CAMERA_MANUAL_EXPOSURE 120
#endif

// 识别板是否优先尝试低 CPU 的 MJPG 采集模式：
// - 1：先尝试 LQ_CAMERA_0CPU_MJPG，若打开失败则自动回退到原高帧率模式
// - 0：直接沿用原 LQ_CAMERA_HIGH_MJPG
#ifndef BW_RECOG_CAMERA_TRY_0CPU_MJPG
#define BW_RECOG_CAMERA_TRY_0CPU_MJPG 0
#endif

// ROI 提取方法：
// 0 = direct_red_quad
// 1 = ipm_square_from_top_edge
#ifndef BW_RECOG_ROI_METHOD
#define BW_RECOG_ROI_METHOD 1
#endif
#pragma endregion

#pragma region 识别板运行时与图传性能参数
// 主循环空闲态睡眠时间（毫秒）：
// 用于“等待按 c 启动”或识别链关闭时降低空转 CPU。
#ifndef BW_RECOG_IDLE_SLEEP_MS
#define BW_RECOG_IDLE_SLEEP_MS 8
#endif

// 主循环活动态睡眠时间（毫秒）：
// 识别链开启后的轻微让步，避免 while(1) 空转把 CPU 持续打满。
#ifndef BW_RECOG_ACTIVE_SLEEP_MS
#define BW_RECOG_ACTIVE_SLEEP_MS 2
#endif

// 普通态便宜预筛频率（毫秒）：
// 仅在 NORMAL 且未锁存成功结果时使用，控制 HSV 红阈值预筛的更新频率。
#ifndef BW_RECOG_NORMAL_PRECHECK_INTERVAL_MS
#define BW_RECOG_NORMAL_PRECHECK_INTERVAL_MS 100
#endif

// 普通态 HSV 红阈值预筛的最小红像素数：
// 低于该值时直接认为“不值得进入重 ROI”。
#ifndef BW_RECOG_NORMAL_PRECHECK_MIN_PIXELS
#define BW_RECOG_NORMAL_PRECHECK_MIN_PIXELS 120
#endif

// 图传发布帧间隔：
// 1 表示每帧都推，2 表示隔一帧推一次。
#ifndef BW_STREAM_PUBLISH_INTERVAL_FRAMES
#define BW_STREAM_PUBLISH_INTERVAL_FRAMES 1
#endif

// 图传最大宽度：
// 大于 0 时，发布前按比例缩放到该宽度以内，减轻 JPEG 编码和网络压力。
#ifndef BW_STREAM_MAX_WIDTH
#define BW_STREAM_MAX_WIDTH 400
#endif

// 图传 JPEG 质量：
// 识别板当前以调试观察为主，适当降低质量可以显著换取更流畅的 MJPEG 帧率。
#ifndef BW_STREAM_JPEG_QUALITY
#define BW_STREAM_JPEG_QUALITY 60
#endif

// 图传输出裁剪下边界（开区间）：
// 仅影响图传发布画面，不影响内部 640x480 原图采集、ROI 提取和分类。
// 当前默认裁掉 y>=280 的部分，和红块搜索主区间保持一致。
#ifndef BW_STREAM_CROP_MAX_Y
#define BW_STREAM_CROP_MAX_Y 280
#endif

// 图传上叠加 ROI 预览的边长（像素）。
#ifndef BW_RECOG_ROI_PREVIEW_SIZE
#define BW_RECOG_ROI_PREVIEW_SIZE 112
#endif
#pragma endregion

#pragma region 双板通信参数
// 状态流模式下，每一帧都发送当前视觉状态；
// 因此不再需要旧的“单事件重复发送若干帧”参数。
// 识别板串口首次初始化失败后，额外重试次数。
// 例如设为 10，则第一次失败后还会再尝试 10 次。
#ifndef BW_BOARD_COMM_INIT_RETRY_TIMES
#define BW_BOARD_COMM_INIT_RETRY_TIMES 10
#endif

// 识别板串口初始化重试间隔（毫秒）。
// 当前按 1s 节奏重试，避免上电阶段设备节点尚未就绪时直接永久放弃。
#ifndef BW_BOARD_COMM_INIT_RETRY_INTERVAL_MS
#define BW_BOARD_COMM_INIT_RETRY_INTERVAL_MS 1000
#endif
#pragma endregion

#pragma region 红块面积阈值
// 非噪声红块最小面积：
// 小于该面积的红色连通域统一视为噪声，不参与后续状态判断。
#ifndef BW_RECOG_RED_MIN_AREA
#define BW_RECOG_RED_MIN_AREA 500
#endif

// 松弛红块候选最小面积：
// 只用于“是否值得先尝试标识板/IPM 路径”这一层；
// 不改变后续砖块阈值与正式 ROI 质量门。
#ifndef BW_RECOG_LOOSE_RED_MIN_AREA
#define BW_RECOG_LOOSE_RED_MIN_AREA 200
#endif

// 砖块面积阈值：
// 在“不满足识别标识”的前提下，面积达到该阈值则回传 b，否则回传 n。
#ifndef BW_RECOG_BRICK_MIN_AREA
#define BW_RECOG_BRICK_MIN_AREA 2000
#endif
#pragma endregion

#pragma region 红框触发几何参数
// 红块搜索带上边界：
// 普通态只在这个纵向窗口里做红块 ROI 搜索，避免全图无意义扫描。
#ifndef BW_RECOG_TRIGGER_SEARCH_Y_MIN
#define BW_RECOG_TRIGGER_SEARCH_Y_MIN 80
#endif

// 红块搜索带下边界（开区间）：
// 当前普通态只在这段纵向区间里做红块搜索与 ROI 提取。
#ifndef BW_RECOG_TRIGGER_SEARCH_Y_MAX
#define BW_RECOG_TRIGGER_SEARCH_Y_MAX 280
#endif

// 红色掩膜允许处理到的最大 y（开区间）：
// roi_runtime_geometry 里会先裁红色掩膜，再找红块；这里与搜索带保持一致。
#ifndef BW_RECOG_RED_MASK_MAX_Y
#define BW_RECOG_RED_MASK_MAX_Y 280
#endif

// 白色参考行：
// 识别板用这条行去估计地面白色横向跨度，帮助限制 ROI 搜索 x 范围。
#ifndef BW_RECOG_WHITE_REFERENCE_ROW_Y
#define BW_RECOG_WHITE_REFERENCE_ROW_Y 280
#endif

// 逆透视横向长方形约束总开关：
// 1 表示要求上/下边明显长于左/右边。
#ifndef BW_RECOG_TRIGGER_IPM_RECT_ENABLE
#define BW_RECOG_TRIGGER_IPM_RECT_ENABLE 1
#endif

// 逆透视后横向长方形的最小宽高比。
#ifndef BW_RECOG_TRIGGER_IPM_MIN_WIDTH_HEIGHT_RATIO
#define BW_RECOG_TRIGGER_IPM_MIN_WIDTH_HEIGHT_RATIO 1.30f
#endif
#pragma endregion

#endif
