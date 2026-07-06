#ifndef _USER_COMMON_H_
#define _USER_COMMON_H_

// 图像侧统一配置入口：
// - 所有图像算法阈值、编译期开关、默认值尽量集中在本文件。
// - 业务 .cc 文件应优先消费这里的语义化常量，避免继续散落局部宏。
// - 当前“绕行/远端接管”只以 image_data.cc + image_process.cc 这条活跃双板链为准。

#pragma region 图像总开关与模式切换
// 环岛单边中线特殊推移总开关：
// 使用位置：image_handle.cc / get_single_side_mid_offset_pixels()。
// 作用：
// - 0：单边中线永远使用默认“半个赛道宽”的偏移，不再区分环岛入环/环内阶段。
// - 1：环岛时允许按 circle_state / circle_direction 使用
//      BW_CIRCLE_IN_OFFSET_RATIO、BW_CIRCLE_RUNNING_OFFSET_RATIO 做特殊推中线。
// 调参建议：
// - 环岛已经很稳且不想再引入额外几何偏置时可关。
// - 若车在入环或环内贴边不够，保留开启。
#ifndef BW_CIRCLE_OFFSET_ENABLE
#define BW_CIRCLE_OFFSET_ENABLE 0
// #define BW_CIRCLE_OFFSET_ENABLE 1
#endif

// pure_angle 预瞄“速度反馈前推”总开关：
// 使用位置：image_handle.cc / preview_shift_from_speed_feedback()。
// 作用：
// - 0：预瞄前推只看当前中线几何，不额外参考目标速度。
// - 1：车速越高，允许在几何前推之外再额外向远处看一点，提升高速提前量。
// 说明：
// - 这里只看 base_target_speed，不直接改控制输出，只改预瞄点位置。
// - 若你想把 pure_angle 手感调得更“纯几何”，先关这个。
#ifndef PUREANGLE_PREVIEW_SPEED_FEEDBACK_ENABLE
#define PUREANGLE_PREVIEW_SPEED_FEEDBACK_ENABLE 0
#endif

// 双板 w/s 是否允许在“锁左/锁右巡线”之外，再短时叠加激进固定 pure_angle。
// 当前主链：
// - 1：收到 w/s 后，先锁单边，再在短窗口内用更激进的固定角覆盖几何输出。
// - 0：收到 w/s 后只锁单边，不再额外打固定角。
// 调参建议：
// - 车已经能靠单边锁线稳定绕行时，可先关掉让动作更顺。
// - 车在目标板前转向不够坚决时，再打开。
#ifndef BW_REMOTE_SIGN_AGGRESSIVE_TURN_ENABLE
#define BW_REMOTE_SIGN_AGGRESSIVE_TURN_ENABLE 0
#endif

// 双板 w/s 激进转角结束后，是否立刻接一个同样时长的反向回摆角。
// 当前行为：
// - 1：主激进角结束后，自动进入反向回摆角。
// - 0：主激进角结束后直接退出固定角覆盖，回到几何 pure_angle。
// 说明：
// - 这里只控制 w/s 绕行动作，不影响 v/u。
// - 回摆仍然会被新远端状态、stale timeout、n/b 覆盖或清空。
#ifndef BW_REMOTE_SIGN_REBOUND_TURN_ENABLE
#define BW_REMOTE_SIGN_REBOUND_TURN_ENABLE 1
#endif

// 双板 w/s 锁边绕行时，强制 path 相对锁定边线“向外”偏移的赛道宽比例。
// 使用位置：image_handle.cc / BuildRemoteFollowOuterLine()。
// 当前语义：
// - 左锁边：以左边线为基准，向赛道左外侧偏移 本比例 * ROADWIDTH。
// - 右锁边：以右边线为基准，向赛道右外侧偏移 本比例 * ROADWIDTH。
// - 这条线会同时同步到 midline.mid（供预瞄/显示）和 midline.path（供控制）。
// 调参建议：
// - 变大：绕行更贴外侧，避让更激进。
// - 变小：更接近原边线，动作更保守。
#ifndef BW_REMOTE_FOLLOW_OUTER_OFFSET_RATIO
#define BW_REMOTE_FOLLOW_OUTER_OFFSET_RATIO 0.05f
#endif

// 普通路段宽度趋势异常时，是否强制退回 MIXED。
// 使用位置：image_midline_process.cc。
// - 1：检测到左右候选中线间距沿前向持续增大/减小，就锁 MIXED。
// - 0：保持当前单边/混合决策，不做这层保护。
#ifndef BW_NORMAL_FORCE_MIXED_BY_WIDTH_TREND_ENABLE
#define BW_NORMAL_FORCE_MIXED_BY_WIDTH_TREND_ENABLE 1
#endif

// pure_angle 预瞄图像行过渡总开关。
// 使用位置：image_handle.cc / pure_angle_apply_preview_transition()。
// - 1：限制 preview_img_y 帧间跳变。
// - 0：预瞄点直接跟目标值，最跟手也最容易抖。
#ifndef PUREANGLE_PREVIEW_TRANSITION_ENABLE
#define PUREANGLE_PREVIEW_TRANSITION_ENABLE 1
#endif

// pure_angle 丢线趋势外推总开关。
// 使用位置：image_process.cc / pure_angle_apply_lost_strategy()。
// - 1：短时缺测时沿上一帧趋势继续推。
// - 0：一旦缺测就更快回保守策略。
#ifndef PUREANGLE_LOST_TREND_ENABLE
#define PUREANGLE_LOST_TREND_ENABLE 1
#endif

// pure_angle 趋势前馈总开关。
// 使用位置：image_process.cc / pure_angle_apply_pre_control()。
// - 1：当转向趋势还在同向加强时，额外补一点前馈角。
// - 0：只用原始 pure_angle。
#ifndef PUREANGLE_PRE_CTRL_ENABLE
#define PUREANGLE_PRE_CTRL_ENABLE 0
#endif

// 斑马线冲线模式。
// - 1：单次冲线。第一次识别到斑马线就冲线，消失后延迟停车。
// - 2：双次冲线。第一次只冲线并恢复正常巡线，第二次再冲线并延迟停车。
// 说明：无论哪种模式，只要当前仍看见斑马线，都会强制冲线并锁 MIXED。
#ifndef BW_ZEBRA_RUSH_MODE
#define BW_ZEBRA_RUSH_MODE 2
#endif

// 图传链默认开关。
// 使用位置：stream_chain.cc / DefaultEnabled。
// - 1：程序默认启动图传。
// - 0：程序默认不启动图传，仍可被命令行覆盖。
#ifndef BW_ENABLE_STREAM
// #define BW_ENABLE_STREAM 1
#define BW_ENABLE_STREAM 0
#endif

// 普通巡线图传底图模式。
// 使用位置：vision_runtime.cc / RenderLineTrackingView。
// - 1：二值图转 BGR 后叠加轨迹，更适合看巡线处理结果。
// - 0：低分辨率彩图叠加轨迹，更适合看原始现场画面。
#ifndef BW_STREAM_LINE_USE_BINARY_VIEW
#define BW_STREAM_LINE_USE_BINARY_VIEW 1
// #define BW_STREAM_LINE_USE_BINARY_VIEW 0
#endif

// 灰度二值化诊断总开关。
// 使用位置：vision_runtime.cc。
// - 1：输出整图/中心/四角灰度统计与 Otsu 阈值。
// - 0：关闭这组纯诊断日志与叠字。
#ifndef BW_GRAY_BIN_DIAG_ENABLE
#define BW_GRAY_BIN_DIAG_ENABLE 0
#endif
#pragma endregion

#pragma region 图像基础参数
// 图像尺寸（全局宏，所有图像处理函数共享）
#define IMAGE_H               (120)   // 图像高度（像素）
#define IMAGE_W               (160)   // 图像宽度（像素）

#define PI32                  (3.1415926535898f) // 圆周率（float）

// 绘图参数（全局宏）
#define DRAWRADIUS            (1)     // 绘图半径（像素）
#define BLACK_IN_GRAY         (0)     // 二值图黑色像素值
#define WHITE_IN_GRAY         (255)   // 二值图白色像素值

// 物理参数（全局宏）
#define PIXPERMETER           (66.45f) // 逆透视后每米像素数
#define ROADWIDTH             (0.45f)  // 赛道宽度（米）

#define PT_MAXLEN             (80)     // 点列最大长度（左右线/中线/路径均使用）
#pragma endregion

#pragma region 图像预处理与寻线基础参数
#define SET_IMAGE_CORE_X      (80)   // 图像核心点X（像素）
#define SET_IMAGE_CORE_Y      (115)  // 图像核心点Y（像素）

#define SEARCH_LINE_START_OFFSET  (20) // 寻线起点横向偏移（像素）
#define SELFADAPT_KERNELSIZE      (7)  // 自适应滤波核尺寸（奇数）
#define FILTER_KERNELSIZE         (7)  // 边线滤波核尺寸（奇数）
#define SELFADAPT_OFFSET          (8)  // 自适应阈值偏移
#define RESAMPLEDIST              (0.02f) // 重采样间距（米）
#define ANGLEDIST                 (0.2f)  // 角度计算“跨度”（米）
#pragma endregion

#pragma region 中线融合基础参数
// 中线（全局宏）
#define MIXED_LINE_DIFF_THRESHOLD_PIX    (0.1f) // 左右中线混合差异阈值（像素）
#define MIXED_POINT_NUM_THRESHOLD        (5)    // 混合中线最少重合点数

// 参与 x 差趋势判定的最少有效配对点数。
#ifndef BW_NORMAL_FORCE_MIXED_MIN_COMMON_POINTS
#define BW_NORMAL_FORCE_MIXED_MIN_COMMON_POINTS 8
#endif
#pragma endregion

#pragma region pure_angle预瞄与路径参数
// -------------------- pure_angle 预瞄与路径 --------------------
// 正常状态默认看 y=90 附近；前方弯越急，会动态把该值往更小处推，形成更强前瞻。
// 当前动态预瞄行的有效工作区间固定收口为 90~70，避免前推过远导致行为过激。
#define PUREANGLE_PREVIEW_BASE_IMAGE_Y    (90)
// pure_angle 预瞄图像行允许推到的最远位置（越小越看远，也越激进）。
// 这里固定到 y=70，不再允许继续推到更远处。
#define PUREANGLE_PREVIEW_MIN_IMAGE_Y     (70)
// 预瞄曲率链的局部曲率计算跨度（点数）。
#define PUREANGLE_PREVIEW_CURV_DIST       (10)
// 预瞄曲率链历史 NMS 窗口大小（点数）：
// 当前 MidLineSuggestPureAnglePreviewImageY 已不再使用 NMS，保留该宏仅为兼容旧调参记录。
#define PUREANGLE_PREVIEW_CURV_NMS_KERNEL (5)
// 预瞄局部转角阈值下限（角度域，单位：deg）：
// 使用 local_curvature_points 得到 1-cos(theta) 后，会先还原成 theta 再参与后续判断。
// 小于该角度时，视为普通直道/缓弯，不触发明显前推。
#define PUREANGLE_PREVIEW_CURVE_LOW       (10.0f)
// 预瞄局部转角阈值上限（角度域，单位：deg）：
// 达到该角度后，认为前方已有明显弯道，预瞄前推到最大。
#define PUREANGLE_PREVIEW_CURVE_HIGH      (25.0f)
// 局部转角单点的物理合理上限（角度域，单位：deg）：
// 预瞄判断只关心“是否已经明显弯起来”，超过该值时对预瞄前推已无额外意义；
// 先做限幅，可压掉边线/中线抖动带来的离谱尖峰。
#define PUREANGLE_PREVIEW_ANGLE_CLAMP_MAX (60.0f)
// 预瞄局部转角链的稳健窗口大小（点数，建议奇数）：
// 不再直接取单点最大值，而是对角度链做短窗平均，只保留“连续一小段都在转”的几何证据。
#define PUREANGLE_PREVIEW_ROBUST_WINDOW   (5)
// 纯局部几何连续映射所允许的最大前推量（图像行）。
#define PUREANGLE_PREVIEW_SHIFT_MAX       (20)
// 特殊几何形态下的附加前推量：连续弯通常比普通弯更需要提前切入。
#define PUREANGLE_PREVIEW_S_CURVE_SHIFT   (12)
// 直角弯/大角度弯下的附加前推量。
#define PUREANGLE_PREVIEW_ANGLE_SHIFT     (18)
// 环岛或长圆弧下的附加前推量。
#define PUREANGLE_PREVIEW_ROUND_SHIFT     (10)
// 中线几何分类里的“最小环岛半径”阈值（逆透视像素）：
// 只用于 pure_angle 动态预瞄的粗粒度赛道判别。
#define MID_TRACK_ROUNDABOUT_MIN_RADIUS_PIX (10.0f)
// 中线几何分类里的“S 弯最小弧段长度”阈值（逆透视像素）：
// 太短的弧段通常只是噪声分段，不参与 S 弯判定。
#define MID_TRACK_S_CURVE_MIN_ARC_LEN_PIX (14.0f)
// 用于“路径并轨到中线”的参考图像行（越小越看远，越大越看近）
#define PATH_BLEND_REF_IMAGE_Y            (80)
#pragma endregion

#pragma region 环岛单边中线偏移参数
#ifndef BW_CIRCLE_IN_OFFSET_RATIO
#define BW_CIRCLE_IN_OFFSET_RATIO 0.3f
#endif

#ifndef BW_CIRCLE_RUNNING_OFFSET_RATIO
#define BW_CIRCLE_RUNNING_OFFSET_RATIO 0.7f
#endif
#pragma endregion

#pragma region pure_angle预瞄过渡参数
// 预瞄图像行单帧最大变化量（pixel/frame）：
// 使用位置：image_handle.cc / pure_angle_apply_preview_transition。
// 作用：限制 preview_img_y 每帧最大漂移速度。
// 调大：预瞄切换更快，可能引入角度跳变。
// 调小：预瞄更稳，但弯道提前量建立更慢。
#ifndef PUREANGLE_PREVIEW_TRANSITION_MAX_STEP
#define PUREANGLE_PREVIEW_TRANSITION_MAX_STEP 4.0f
#endif

// 预瞄图像行过渡滤波系数：
// 使用位置：image_handle.cc / pure_angle_apply_preview_transition。
// 作用：对限速后的预瞄目标再做一阶低通。
// 越接近 1.0 越跟手；越小越平滑。
// 推荐：先固定 MAX_STEP，再微调该项。
#ifndef PUREANGLE_PREVIEW_TRANSITION_ALPHA
#define PUREANGLE_PREVIEW_TRANSITION_ALPHA 0.70f
#endif

// pure_angle 大角度抑制启动阈值（deg）：
// 使用位置：image_handle.cc / CalculatePureAngleFromPath。
// 作用：小于等于该阈值时，pure_angle 直接输出原始几何角。
// 大于该阈值时，进入平滑饱和区，逐步逼近 PUREANGLE_PROGRESSIVE_MAX_ABS_DEG。
#ifndef PUREANGLE_RAW_LIMIT_THRESHOLD_DEG
#define PUREANGLE_RAW_LIMIT_THRESHOLD_DEG 35.0f
#endif

// pure_angle 渐进饱和上限（deg）：
// 使用位置：image_handle.cc / CalculatePureAngleFromPath。
// 作用：作为大角度抑制后的理论极限值，只允许逼近，不允许达到或超过。
// 要求：必须严格大于 PUREANGLE_RAW_LIMIT_THRESHOLD_DEG。
#ifndef PUREANGLE_PROGRESSIVE_MAX_ABS_DEG
#define PUREANGLE_PROGRESSIVE_MAX_ABS_DEG 45.0f
#endif

// 双板 w/s 锁边绕行时，差速防反转保留裕量：
// 使用位置：PID.cc / 仅 remote_follow_locked 分支。
// 作用：
// - 当前差速链里，factor=+1 时右轮目标速度会刚好压到 0，factor=-1 时左轮会刚好压到 0。
// - 本参数会把绕行态 factor 限到 [- (1-margin), +(1-margin)]，避免内侧轮被打成负速反转。
// 调小：
// - 更接近极限，绕行更激进。
// 调大：
// - 留量更大，更稳，但内侧减速不会那么狠。
#ifndef BW_REMOTE_FOLLOW_NO_REVERSE_FACTOR_MARGIN
#define BW_REMOTE_FOLLOW_NO_REVERSE_FACTOR_MARGIN 0.02f
#endif
#pragma endregion

#pragma region pure_angle丢线补偿参数
// 丢线趋势外推最长持续帧数：
// 使用位置：image_process.cc / pure_angle_apply_lost_strategy。
// 作用：决定外推保持多久后转入衰减回零。
// 调大：连续缺测时更激进。
// 调小：更保守，更快放弃旧趋势。
#ifndef PUREANGLE_LOST_TREND_FRAMES
#define PUREANGLE_LOST_TREND_FRAMES 15
#endif

// 丢线趋势外推的单帧最大步长（deg/frame）：
// 使用位置：image_process.cc / clip_step。
// 作用：限制“上一帧趋势”被放大后的最大角速度。
// 调大：补偿更猛，但更容易暴冲。
// 调小：更稳，但连续急弯维持能力下降。
#ifndef PUREANGLE_LOST_MAX_STEP_DEG
#define PUREANGLE_LOST_MAX_STEP_DEG 6.0f
#endif

// 丢线趋势外推斜率衰减系数：
// 使用位置：image_process.cc / pure_angle_apply_lost_strategy。
// 作用：每帧对外推趋势 rate 做衰减，避免无限保持旧趋势。
// 越接近 1.0 越“敢冲”；越小越快收敛。
#ifndef PUREANGLE_LOST_RATE_DECAY
#define PUREANGLE_LOST_RATE_DECAY 0.85f
#endif

// 长时间丢线后向 0 度回归的衰减系数：
// 使用位置：image_process.cc / pure_angle_apply_lost_strategy。
// 作用：在超过 LOST_TREND_FRAMES 后，把输出角逐步拉回中性。
// 越接近 1.0，回正越慢；越小，回正越快。
#ifndef PUREANGLE_LOST_RETURN_ZERO_DECAY
#define PUREANGLE_LOST_RETURN_ZERO_DECAY 0.92f
#endif

// 回线后的软切换帧数：
// 使用位置：image_process.cc / pure_angle_apply_lost_strategy。
// 作用：从外推角平滑过渡回真实测量，避免突然切回。
// 调大：回线更柔和。
// 调小：更直接，但可能在恢复测量时出现跳变。
#ifndef PUREANGLE_REACQ_BLEND_FRAMES
#define PUREANGLE_REACQ_BLEND_FRAMES 4
#endif
#pragma endregion

#pragma region pure_angle趋势前馈参数
// 趋势前馈启动角阈值（deg）：
// 使用位置：image_process.cc / pure_angle_apply_pre_control。
// 作用：只有已经进入明显转弯区时才允许前馈。
// 调大：介入更晚。
// 调小：介入更早，也更容易放大轻微抖动。
#ifndef PUREANGLE_PRE_CTRL_START_DEG
#define PUREANGLE_PRE_CTRL_START_DEG 6.0f
#endif

// 趋势前馈启动的最小角度增量阈值（deg/frame）：
// 使用位置：image_process.cc / pure_angle_apply_pre_control。
// 作用：判断当前转向趋势是否还在加强。
// 调大：更难触发。
// 调小：更易触发，但更容易把噪声当趋势。
#ifndef PUREANGLE_PRE_CTRL_DELTA_START_DEG
#define PUREANGLE_PRE_CTRL_DELTA_START_DEG 1.0f
#endif

// 趋势前馈增益：
// 使用位置：image_process.cc / pure_angle_apply_pre_control。
// 作用：extra = gain * delta。
// 调大：转向更猛。
// 调小：更稳，更接近原始 pure_angle。
#ifndef PUREANGLE_PRE_CTRL_GAIN
#define PUREANGLE_PRE_CTRL_GAIN 0.7f
#endif

// 趋势前馈最大额外补偿角（deg）：
// 使用位置：image_process.cc / pure_angle_apply_pre_control。
// 作用：限制单帧趋势前馈的最大放大量。
// 调大：连续急弯更激进。
// 调小：整体更稳，代价是补偿上限更低。
#ifndef PUREANGLE_PRE_CTRL_MAX_EXTRA_DEG
#define PUREANGLE_PRE_CTRL_MAX_EXTRA_DEG 4.0f
#endif
#pragma endregion

#pragma region 斑马线检测与停车参数
// -------------------- 斑马线检测与停车 --------------------
// 近距离检测长度（点数）：
// 使用位置：element/zebra.cc。
// 作用：限制斑马线在近车区域内的检测窗口。
#ifndef NEAR_DETECT_MAX
#define NEAR_DETECT_MAX (30)
#endif

// 单行扫描时，忽略首尾色块后，黑色块和白色块各自至少需要这么多个。
// 使用位置：element/zebra.cc。
// 设计口径：赛道 45cm、斑马线 2.5cm 黑白交替，理论上黑白各约 9 个；
// 实战中放宽到各 >=6 即可认定有明显斑马线结构。
#ifndef ZEBRA_MIN_COLOR_RUNS
#define ZEBRA_MIN_COLOR_RUNS (6)
#endif

// 单行扫描时，忽略首尾色块后，至少需要这么多次黑白切换。
// 使用位置：element/zebra.cc。
// 等价于要求赛道横向上出现足够密集的黑白交替。
#ifndef ZEBRA_MIN_SWITCHES
#define ZEBRA_MIN_SWITCHES (11)
#endif

// 一帧内至少需要命中这么多条扫描线，才最终判定为斑马线。
// 使用位置：element/zebra.cc。
// 调大：更稳，误检更少。
// 调小：更灵敏。
#ifndef ZEBRA_MIN_HIT_ROWS
#define ZEBRA_MIN_HIT_ROWS (2)
#endif

// 从近处边线点列里按多大步长抽取一条扫描线。
// 使用位置：element/zebra.cc。
// 调大：计算更省，但覆盖更稀。
// 调小：覆盖更密，但更敏感。
#ifndef ZEBRA_ROW_SAMPLE_STEP
#define ZEBRA_ROW_SAMPLE_STEP (2)
#endif

// 扫描时离左右边线各裁掉多少像素，避免边线附近噪声影响。
// 使用位置：element/zebra.cc。
#ifndef ZEBRA_EDGE_MARGIN
#define ZEBRA_EDGE_MARGIN (2)
#endif

// 当另一侧边线没有同 y 点时，允许用最近点兜底的最大 y 偏差。
// 使用位置：element/zebra.cc。
#ifndef ZEBRA_EDGE_MATCH_Y_TOL
#define ZEBRA_EDGE_MATCH_Y_TOL (2)
#endif

// 斑马线解锁冷却时间（毫秒）：
// 使用位置：image_process.cc。
// 作用：手动解除 zebra_stop 后，短时间内不允许再次触发。
#ifndef ZEBRA_COOLDOWN_MS
#define ZEBRA_COOLDOWN_MS 1200
#endif

// 斑马线冲线时对基础速度施加的倍率。
// 作用位置：
// - image_process.cc 负责在冲线期间锁状态、维持 MIXED。
// - PID.cc 的活跃 Control_Loop 负责把 base_target_speed 乘这个倍率。
// 调大：更敢冲，但停车前的距离需求也更大。
// 调小：更稳，但更像“慢速压线”而不是“冲线”。
#ifndef BW_ZEBRA_RUSH_SPEED_RATIO
#define BW_ZEBRA_RUSH_SPEED_RATIO 1.25f
#endif

// 检测到斑马线后延迟停车的时间（毫秒）：
// 使用位置：image_process.cc。
// 作用：
// - 单次冲线模式：第一次斑马线消失后，等待这么久再停车。
// - 双次冲线模式：第二次斑马线消失后，等待这么久再停车。
// 调大：车会冲得更深。
// 调小：更早刹停。
#ifndef ZEBRA_STOP_DELAY_MS
#define ZEBRA_STOP_DELAY_MS 1000
#endif

// 双次冲线模式下，第一次命中斑马线后的“新判定休眠”时间（毫秒）。
// 使用位置：image_process.cc / update_zebra_rush_state()。
// 作用：
// - 只在 BW_ZEBRA_RUSH_MODE >= 2 且刚完成第一次命中计数后生效。
// - 休眠期内不再接受新的斑马线上升沿，避免同一条斑马线因为多帧抖动/短时漏检被误当成第二次经过。
// 说明：
// - 这不会打断当前这一次冲线；它只阻止“第一次结束后立刻又被重新计数”。
// - 调大：更不容易把同一条斑马线算成两次，但第二次真实斑马线必须离第一次更远。
// - 调小：更灵敏，但更容易被连续帧误触发第二次。
#ifndef BW_ZEBRA_DOUBLE_FIRST_SLEEP_MS
#define BW_ZEBRA_DOUBLE_FIRST_SLEEP_MS 5000
#endif

// 历史保留：曾用特殊 pure_angle 表达停车。
// 当前不再使用，仅保留兼容外部旧引用。
#ifndef ZEBRA_STOP_ANGLE
#define ZEBRA_STOP_ANGLE 111.11f
#endif
#pragma endregion

#pragma region 双板通信与绕行动作参数
// 当前双板绕行主链 owner：
// - 状态适配：image_data.cc / image_remote_recognition_apply_state()
// - 图像接管：image_process.cc / img_processing()
// 当前真实行为不是旧 TargetHandler 分阶段 pure_angle 绕行，而是：
// - w：锁左巡线 + 可选短时激进左转 pure_angle
// - s：锁右巡线 + 可选短时激进右转 pure_angle
// - v：vehicle 特殊巡线，短时保持收到包当下 pure_angle
// - u：同 v，但同时把基础速度按比例降下来
// - b：若正处于 CIRCLE_BEGIN / CIRCLE_IN，则打掉环岛状态机并回到 MIXED
//
// 下列参数只服务于这条“当前活跃”的双板接管链。
// 旧 TargetHandler 分阶段绕行参数已移出 common.h，不再作为公共调参入口暴露。

// 收到 w/s 后，当“进入激进角当下的 pure_angle”与“绕行方向”同向时，额外叠加的激进角（度）。
// 例子：
// - 当前已经左拐(positive) + 继续左绕(w) -> 用这组
// - 当前已经右拐(negative) + 继续右绕(s) -> 也用这组
// 说明：
// - 这里只作用于主激进角，不作用于反向回摆角。
#ifndef BW_REMOTE_SIGN_AGGRESSIVE_ABS_PURE_ANGLE
#define BW_REMOTE_SIGN_AGGRESSIVE_ABS_PURE_ANGLE 5.0f
#endif

// 收到 w/s 后，当“进入激进角当下的 pure_angle”与“绕行方向”反向时，额外叠加的激进角（度）。
// 例子：
// - 当前已经左拐(positive) + 右绕(s) -> 用这组
// - 当前已经右拐(negative) + 左绕(w) -> 也用这组
#ifndef BW_REMOTE_SIGN_AGGRESSIVE_OPPOSITE_ABS_PURE_ANGLE
#define BW_REMOTE_SIGN_AGGRESSIVE_OPPOSITE_ABS_PURE_ANGLE 20.0f
#endif

// 主激进角叠加完成后的统一限幅（度）。
// 作用：
// - 主激进角 = entry_yaw + route_sign * add_deg
// - 上式算完后再夹到 [-limit, +limit]
// - 仅限制主激进角，不限制反向回摆固定角
#ifndef BW_REMOTE_SIGN_AGGRESSIVE_OUTPUT_LIMIT_DEG
#define BW_REMOTE_SIGN_AGGRESSIVE_OUTPUT_LIMIT_DEG 25.0f
#endif

// 当进入激进角当下的原始偏航角绝对值不大于该阈值时，不使用“entry_yaw + 叠加量”。
// 而是直接切到固定主激进角，见 BW_REMOTE_SIGN_AGGRESSIVE_SMALL_YAW_FIXED_DEG。
#ifndef BW_REMOTE_SIGN_AGGRESSIVE_SMALL_YAW_THRESHOLD_DEG
#define BW_REMOTE_SIGN_AGGRESSIVE_SMALL_YAW_THRESHOLD_DEG 10.0f
#endif

// 小偏航场景下使用的固定主激进角（度）。
// 输出形式：
// - 左绕(w) 固定为 +本值
// - 右绕(s) 固定为 -本值
// 说明：
// - 这条规则只作用于主激进角，不作用于反向回摆角。
#ifndef BW_REMOTE_SIGN_AGGRESSIVE_SMALL_YAW_FIXED_DEG
#define BW_REMOTE_SIGN_AGGRESSIVE_SMALL_YAW_FIXED_DEG 20.0f
#endif

// w/s 激进角阶段的基础速度倍率。
// 作用：
// - 只要当前还处在“主激进角 / 反向回摆角”任一阶段，PID 基础速度都会乘这个比例。
// - 作用对象仅限 w/s 激进角链，不影响 u 的慢速、v 的 hold_yaw、斑马线冲线倍率。
// 调参建议：
// - 太小：绕行动作更稳，但速度掉得太狠可能导致动作发钝。
// - 太大：激进角期间速度保留更多，但更容易甩尾或推过头。
#ifndef BW_REMOTE_SIGN_AGGRESSIVE_SPEED_RATIO
#define BW_REMOTE_SIGN_AGGRESSIVE_SPEED_RATIO 1.0f
#endif

// 反向回摆角相对于“主激进角基准值”的幅度比例。
// 当前语义：
// - 反向回摆角绝对值 = BW_REMOTE_SIGN_AGGRESSIVE_ABS_PURE_ANGLE * 本比例。
// - 回摆仍然是固定角，不叠加入场偏航角。
// 说明：
// - 这里只改“反向回摆角”幅度，不改主激进角幅度。
// - 若回摆明显过猛，可继续调小；若回摆几乎没有拉回效果，可略微调大。
#ifndef BW_REMOTE_SIGN_REBOUND_RATIO
#define BW_REMOTE_SIGN_REBOUND_RATIO 1.0f
#endif

// 收到 w/s 后，激进固定转角允许持续的最长时间（毫秒）。
// 当前退出语义：
// - 正向激进角：当几何 pure_angle 已经反向、或当前锁定侧边线丢失、或超时，就结束。
// - 反向回摆角：当当前锁定侧边线重新找回、或超时，就结束。
// - 任一阶段都可能被新远端状态、n/b 覆盖，或 stale timeout 整体复位提前打断。
#ifndef BW_REMOTE_SIGN_AGGRESSIVE_MAX_MS
#define BW_REMOTE_SIGN_AGGRESSIVE_MAX_MS 10000
#endif

// 远端识别状态总过期时间（毫秒）。
// 作用：
// - 识别板心跳若长时间收不到，这里会整体清空远端接管状态。
// 建议：
// - 需要始终大于识别板心跳周期。
// - 太小容易误掉线；太大则旧状态残留更久。
#ifndef BW_REMOTE_STATE_STALE_MS
#define BW_REMOTE_STATE_STALE_MS 300
#endif

// 收到 v/u 后，保持“收到包当下 pure_angle”的持续时间（毫秒）。
// 作用：
// - vehicle 特殊巡线一旦在局部起线失败，会短时回退到这个锁存角。
// 调大：更像“顶着原朝向继续冲过去”，对短时丢线更稳。
// 调小：更快回普通巡线，但也更容易在 vehicle 阶段抖动。
#ifndef BW_REMOTE_VEHICLE_HOLD_MS
#define BW_REMOTE_VEHICLE_HOLD_MS 1000
#endif

// vehicle 特殊巡线“跳黑块找线”时，core 左右参与黑块判定的半宽。
// 作用位置：
// - image_process.cc / vehicle_window_all_white()
// 调大：更容易把中央附近黑块也当成“仍被挡住”，起线更保守。
// 调小：更容易提前放行起线。
#ifndef BW_REMOTE_VEHICLE_SKIP_HALF_WIDTH
#define BW_REMOTE_VEHICLE_SKIP_HALF_WIDTH 2
#endif

// 收到 u 后，对基础速度施加的比例倍率。
// 作用：
// - 当前活跃 Control_Loop 会把 base_target_speed 乘这个比例。
// - 只有在斑马线冲线未激活时才会被应用；冲线优先级更高。
// 调大：u 阶段更快，更接近普通速度。
// 调小：u 阶段更稳，但车更慢。
#ifndef BW_REMOTE_U_SLOWDOWN_RATIO
#define BW_REMOTE_U_SLOWDOWN_RATIO 0.3f
#endif
#pragma endregion

#pragma region 图传开关与图传模式切换
#pragma endregion

#pragma region 灰度二值化诊断参数
// 灰度诊断采样块边长（像素）：
// 使用位置：vision_runtime.cc。
// 作用：中心和四角都用同样大小的小块统计均值。
#ifndef BW_GRAY_BIN_DIAG_PATCH_SIZE
#define BW_GRAY_BIN_DIAG_PATCH_SIZE 16
#endif

// 灰度诊断日志最小输出间隔（毫秒）：
// 使用位置：vision_runtime.cc。
// 作用：避免每帧都刷屏；图传开启时仍会每帧叠加当前统计值。
#ifndef BW_GRAY_BIN_DIAG_LOG_INTERVAL_MS
#define BW_GRAY_BIN_DIAG_LOG_INTERVAL_MS 400
#endif
#pragma endregion

#pragma region 历史保留切换参数
#define FRAMENONE             (3) // 预留：状态保持帧数（未使用/保留）
#define FRAMETOLEFT           (5) // 预留：左切换保护帧（未使用/保留）
#define FRAMETORIGHT          (5) // 预留：右切换保护帧（未使用/保留）
#pragma endregion

#pragma region 车辆属性参数
// 车辆属性（全局宏）
#define DISTANCE_FROM_VIEW_TO_CAR   (0.23f) // 图像(SET_IMAGE_CORE)到车轴距离（米）
#define WIDTH_OF_CAR                (0.18f) // 车体宽度（米）
#pragma endregion

#pragma region 图像特征量参数
// 特征量（全局宏）
// 角点
#define ANGLE_THRESHOLD_get_corners_conf_max  (120.0f) // 角点检测置信度上限（度）
#define ANGLE_THRESHOLD_get_corners_conf_min  (60.0f)  // 角点检测置信度下限（度）
#define ID_THRESHOLD_get_corners_near_detect  (15)     // 角点近距离索引上限
// 曲线
#define CURVE_THRESHOLD                        (10.0f * PI32 / 180.0f) // 曲线判断角度阈值（弧度）
// 直线
#define ANGLE_THRESHOLD_is_straight            (8.0f) // 直线判断角度阈值（度）
#pragma endregion

#pragma region 左右线切换保留参数
// 左右线切换点数差阈值（全局宏）
#define PTS_THRESHOLD_follow_left (0) // 左跟线点数差阈值（保留）
#pragma endregion

#pragma region 元素判定参数
// 元素判定（全局宏）
// 双角点切换十字状态的 id 阈值
#define ID_THRESHOLD_crossing_state_change (20) // 十字判定角点索引阈值
// 环岛保护帧数
#define FRAME_THRESHOLD_roundabout_protect_frame (3) // 环岛状态保护帧
// 环岛或斜入十字判定帧数
#define FRAME_THRESHOLD_roundabout_or_crossing_frame (3) // 元素投票确认帧
// 斜入十字保护帧数
#define FRAME_THRESHOLD_one_corner_crossing_protect_frame (3) // 十字保护帧
// 角点对侧直线性窗口判断阈值
#define WINDOW_THRESHOLD_roundabout_opposite_straightness (10) // 对侧直线性窗口半宽（点数）
#pragma endregion

#pragma region 元素状态机帧阈值参数
// 元素帧数阈值（全局宏）
// 十字路口丢线帧数阈值
#define FRAME_THRESHOLD_crossing_lost_line_counter (2) // 十字入内阶段允许的连续丢线帧
// 环岛入环丢线阈值
#define FRAME_THRESHOLD_roundabout_begin_to_in_lost_line_counter (2) // BEGIN->IN 丢线阈值
// 环岛入环重新找线阈值
#define FRAME_THRESHOLD_roundabout_begin_to_in_found_line_counter (2) // BEGIN->IN 找线阈值
// 环岛出环岛角点连续判定阈值
#define FRAME_THRESHOLD_roundabout_running_to_out_corner_counter (0) // RUNNING->OUT 角点连续帧
// 环岛结束丢线阈值
#define FRAME_THRESHOLD_roundabout_end_lost_line_counter (2) // END 退出丢线阈值
// 环岛结束重新找线阈值
#define FRAME_THRESHOLD_roundabout_end_found_line_counter (2) // END 退出找线阈值
#pragma endregion

#pragma region 赛道类型角度阈值参数
// 赛道类型角度阈值（全局宏）
// 曲线判断角度
#define ANGLE_THRESHOLD_track_type_curve       (15.0f) // 曲线判定阈值（度）
// 急弯判断角度
#define ANGLE_THRESHOLD_track_type_sharp_curve (35.0f) // 急弯判定阈值（度）
// 急弯保护
#define FRAME_THRESHOLD_sharp_curve_protect_frame (25) // 急弯保护帧
#pragma endregion

#endif /* USER_CAMERA_COMMON_H_ */
