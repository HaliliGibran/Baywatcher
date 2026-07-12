#ifndef _USER_COMMON_H_
#define _USER_COMMON_H_

// 图像侧统一配置入口：
// - 所有图像算法阈值、编译期开关、默认值尽量集中在本文件。
// - 业务 .cc 文件应优先消费这里的语义化常量，避免继续散落局部宏。
// - 当前“绕行/远端接管”只以 image_data.cc + image_process.cc 这条活跃双板链为准。

#pragma region 图像总开关与模式切换

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

#pragma endregion

#pragma region 图像基础参数
// 图像尺寸（全局宏，所有图像处理函数共享）
#define IMAGE_H               (120)   // 图像高度（像素）
#define IMAGE_W               (160)   // 图像宽度（像素）

#define PI32                  (3.1415926535898f) // 圆周率（float）

// 二值图像素值（全局宏）
#define BLACK_IN_GRAY         (0)     // 二值图黑色像素值
#define WHITE_IN_GRAY         (255)   // 二值图白色像素值

// 物理参数（全局宏）
#define PIXPERMETER           (66.45f) // 逆透视后每米像素数
#define ROADWIDTH             (0.45f)  // 赛道宽度（米）

#define PT_MAXLEN             (80)     // 近线/常规中线/路径点列最大长度
#pragma endregion

#pragma region 图像预处理与寻线基础参数
#define SET_IMAGE_CORE_X      (80)   // 图像核心点X（像素）
#define SET_IMAGE_CORE_Y      (115)  // 图像核心点Y（像素）

#define SEARCH_LINE_START_OFFSET  (20) // 寻线起点横向偏移（像素）
#define FILTER_KERNELSIZE         (7)  // 边线滤波核尺寸（奇数）
#define RESAMPLEDIST              (0.02f) // 重采样间距（米）
#define ANGLEDIST                 (0.2f)  // 角度计算“跨度”（米）
#pragma endregion

#pragma region pure_angle预瞄与路径参数
// pure_angle 默认预瞄图像行；数值越大越看近，越小越看远。
#define PUREANGLE_PREVIEW_BASE_IMAGE_Y    (90)

// pure_angle 预瞄允许前推到的最远图像行。
#define PUREANGLE_PREVIEW_MIN_IMAGE_Y     (70)

// 用中线局部转角决定预瞄前推量：跨度越大越抗噪，但响应越慢。
#define PUREANGLE_PREVIEW_CURV_DIST       (10)

// 局部转角小于该值时不明显前推；达到 HIGH 后前推到最大。
#define PUREANGLE_PREVIEW_CURVE_LOW       (10.0f)
#define PUREANGLE_PREVIEW_CURVE_HIGH      (25.0f)

// 单点局部转角限幅，压掉边线/中线抖动造成的离谱尖峰。
#define PUREANGLE_PREVIEW_ANGLE_CLAMP_MAX (60.0f)

// 局部转角稳健窗口；建议奇数。窗口内取中位数，再取整段最大值。
#define PUREANGLE_PREVIEW_ROBUST_WINDOW   (5)

// 局部转角映射到预瞄图像行的最大前推量。
#define PUREANGLE_PREVIEW_SHIFT_MAX       (20)

// path 从 core 渐进并入中线时，选择并轨完成点的参考图像行。
#define PATH_BLEND_REF_IMAGE_Y            (80)
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

#pragma endregion

#pragma region 双板通信与绕行动作参数
// 当前双板绕行主链 owner：
// - 状态适配：image_data.cc / image_remote_recognition_apply_state()
// - 图像接管：image_process.cc / img_processing()
// 当前真实行为不是旧 TargetHandler 分阶段 pure_angle 绕行，而是：
// - w：锁左巡线
// - s：锁右巡线
// - v：vehicle 特殊巡线，短时保持收到包当下 pure_angle
// - u：只降低基础速度，不改变巡线模式、不保持偏航角，也不冻结运行板元素状态机
// - c：软件盲盒色布停车，只限制速度到极低值，不冻结状态机
// - bl/br：若正处于 CIRCLE_BEGIN / CIRCLE_IN，则短时打掉一次环岛状态机，并把中线向另一侧额外侧移
// - b：兼容旧无侧别砖块码，只做短时环岛压制，不做中线侧移
//
// 下列参数只服务于这条“当前活跃”的双板接管链。
// 旧 TargetHandler 分阶段绕行参数已移出 common.h，不再作为公共调参入口暴露。

// ===== w/s 锁边绕行：目标线与平均速度参数 =====

// 外绕/普通锁边时，强制 path 相对锁定边线向外偏移的赛道宽比例。
// 使用位置：image_handle.cc / BuildRemoteFollowOuterLine()。
// 左锁边向赛道左外侧偏移；右锁边向赛道右外侧偏移。
// 这条线会同时同步到 midline.mid（供预瞄/显示）和 midline.path（供控制）。
// 调大：绕行更贴外侧，避让更激进；调小：更接近原边线，动作更保守。
#ifndef BW_REMOTE_FOLLOW_OUTER_OFFSET_RATIO
#define BW_REMOTE_FOLLOW_OUTER_OFFSET_RATIO 0.08f
#endif

// 内绕判定阈值（度）：左弯 + 锁左边线、右弯 + 锁右边线，且弯向角绝对值超过本阈值时视为内绕。
#ifndef BW_REMOTE_FOLLOW_INNER_CURVE_THRESHOLD_DEG
#define BW_REMOTE_FOLLOW_INNER_CURVE_THRESHOLD_DEG 10.0f
#endif

// 内绕时，锁定边线向外推的赛道宽比例。
// 内绕比外绕更容易撞目标板，因此默认比 BW_REMOTE_FOLLOW_OUTER_OFFSET_RATIO 大。
#ifndef BW_REMOTE_FOLLOW_INNER_OFFSET_RATIO
#define BW_REMOTE_FOLLOW_INNER_OFFSET_RATIO 0.10f
#endif

// 外绕时左右轮目标平均速度的倍率。
// 只缩放两轮目标速度的平均值，左右轮差速量保持不变；1.0 表示不减速。
#ifndef BW_REMOTE_FOLLOW_OUTER_AVERAGE_SPEED_RATIO
#define BW_REMOTE_FOLLOW_OUTER_AVERAGE_SPEED_RATIO 1.0f
#endif

// 内绕时左右轮目标平均速度的倍率。
// 只缩放两轮目标速度的平均值，左右轮差速量保持不变。
#ifndef BW_REMOTE_FOLLOW_INNER_AVERAGE_SPEED_RATIO
#define BW_REMOTE_FOLLOW_INNER_AVERAGE_SPEED_RATIO 0.8f
#endif

// 内外绕共用的小角解除减速阈值（度）。
// abs(pure_angle) 小于等于本值时，平均速度倍率临时恢复为 1.0，差速仍按普通方向链计算。
#ifndef BW_REMOTE_FOLLOW_SPEED_RELEASE_ANGLE_DEG
#define BW_REMOTE_FOLLOW_SPEED_RELEASE_ANGLE_DEG 3.0f
#endif

// ===== 环岛 RUNNING 内外绕专用参数 =====
// 环岛内外绕判定不再依赖曲率符号：
// - 左环岛锁左边线为内绕；
// - 右环岛锁右边线为内绕。
// - 左环岛锁右边线、右环岛锁左边线为外绕。

// 环岛内绕时，锁定边线向外推的赛道宽比例。
// 初值与普通内绕一致，调参时只修改本宏，不影响普通弯道内绕。
#ifndef BW_REMOTE_FOLLOW_CIRCLE_INNER_OFFSET_RATIO
#define BW_REMOTE_FOLLOW_CIRCLE_INNER_OFFSET_RATIO 0.05f
#endif

// 环岛内绕时左右轮目标平均速度倍率，只缩放平均速度，不改变差速量。
#ifndef BW_REMOTE_FOLLOW_CIRCLE_INNER_AVERAGE_SPEED_RATIO
#define BW_REMOTE_FOLLOW_CIRCLE_INNER_AVERAGE_SPEED_RATIO 0.8f
#endif

// 环岛内绕的小角解除减速阈值（度）。
#ifndef BW_REMOTE_FOLLOW_CIRCLE_INNER_SPEED_RELEASE_ANGLE_DEG
#define BW_REMOTE_FOLLOW_CIRCLE_INNER_SPEED_RELEASE_ANGLE_DEG 3.0f
#endif

// 环岛外绕时，锁定边线向外推的赛道宽比例。
// 初值与普通外绕一致，调参时只修改本宏，不影响普通路段外绕。
#ifndef BW_REMOTE_FOLLOW_CIRCLE_OUTER_OFFSET_RATIO
#define BW_REMOTE_FOLLOW_CIRCLE_OUTER_OFFSET_RATIO 0.03f
#endif

// 环岛外绕时左右轮目标平均速度倍率，只缩放平均速度，不改变差速量。
#ifndef BW_REMOTE_FOLLOW_CIRCLE_OUTER_AVERAGE_SPEED_RATIO
#define BW_REMOTE_FOLLOW_CIRCLE_OUTER_AVERAGE_SPEED_RATIO 1.0f
#endif

// 环岛外绕的小角解除减速阈值（度）。
#ifndef BW_REMOTE_FOLLOW_CIRCLE_OUTER_SPEED_RELEASE_ANGLE_DEG
#define BW_REMOTE_FOLLOW_CIRCLE_OUTER_SPEED_RELEASE_ANGLE_DEG 3.0f
#endif

// ===== 环岛基础速度倍率 =====
// 作用顺序：base_target_speed 先乘当前环岛阶段倍率，后续 u 减速、
// 速度上限和绕行平均速度倍率继续作用在这个已降低的基础速度上。
// BEGIN/END/NONE 不施加本组倍率。
#ifndef BW_CIRCLE_IN_BASE_SPEED_RATIO
#define BW_CIRCLE_IN_BASE_SPEED_RATIO 0.9f
#endif

#ifndef BW_CIRCLE_RUNNING_BASE_SPEED_RATIO
#define BW_CIRCLE_RUNNING_BASE_SPEED_RATIO 0.7f
#endif

#ifndef BW_CIRCLE_OUT_BASE_SPEED_RATIO
#define BW_CIRCLE_OUT_BASE_SPEED_RATIO 0.9f
#endif

// ===== 远端状态保持与减速参数 =====

// 环岛识别门控总开关。
// 开启后，运行板在环岛候选、BEGIN、IN、OUT、END 阶段阻断识别板；
// 普通赛道和 CIRCLE_RUNNING 阶段允许识别。
#ifndef BW_CIRCLE_RECOGNITION_GATE_ENABLE
#define BW_CIRCLE_RECOGNITION_GATE_ENABLE 1
#endif

// 环岛候选消失或环岛退出后的门控保持时间（毫秒），用于抑制边界抖动。
#ifndef BW_CIRCLE_RECOGNITION_GATE_HOLD_MS
#define BW_CIRCLE_RECOGNITION_GATE_HOLD_MS 150
#endif

// 原始环岛单角点候选需要同方向连续出现的帧数，确认后才提前阻断识别板。
// 2 帧可过滤十字左右角点不同步产生的单帧假环岛，同时仍早于 3 帧环岛状态机确认。
#ifndef BW_CIRCLE_RECOGNITION_GATE_CANDIDATE_CONFIRM_FRAMES
#define BW_CIRCLE_RECOGNITION_GATE_CANDIDATE_CONFIRM_FRAMES 2
#endif

// 运行板反向发送 ALLOW/BLOCK 门控心跳的周期（毫秒）。
#ifndef BW_CIRCLE_RECOGNITION_GATE_HEARTBEAT_MS
#define BW_CIRCLE_RECOGNITION_GATE_HEARTBEAT_MS 50
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

// 收到 b 后压制环岛状态机的短时窗口（毫秒）。
// 作用：
// - b 只用于打断一次砖块附近的环岛误判，不作为停车或长期锁状态命令。
// - 连续 b 不会无限续期；离开 b 再重新进入 b 才会再次触发。
// 调大：砖块附近更不容易重新进环岛，但更容易短时影响普通状态机。
// 调小：更快恢复普通状态机。
#ifndef BW_REMOTE_BRICK_BLOCK_MS
#define BW_REMOTE_BRICK_BLOCK_MS 120
#endif

// 收到 bl/br 后，中线在送入 path 前向红砖反方向额外平移的赛道宽比例。
// bl：砖在左侧，中线向右平移；br：砖在右侧，中线向左平移。
#ifndef BW_REMOTE_BRICK_AVOID_OFFSET_RATIO
#define BW_REMOTE_BRICK_AVOID_OFFSET_RATIO 0.25f
#endif

// [开关] 收到 u 后是否按比例降低基础目标速度。
#ifndef BW_REMOTE_U_SLOWDOWN_ENABLE
#define BW_REMOTE_U_SLOWDOWN_ENABLE 1
#endif

// 收到 u 后的基础速度倍率，仅在 BW_REMOTE_U_SLOWDOWN_ENABLE=1 时生效。
// 0.3 表示 base_target_speed 降为原来的 30%，不再作为绝对速度上限。
// 调大：u 阶段更快，更接近普通速度。
// 调小：u 阶段更稳，但车更慢。
#ifndef BW_REMOTE_U_SLOWDOWN_RATIO
#define BW_REMOTE_U_SLOWDOWN_RATIO 0.8f
#endif

// 收到 c 后的强制停车级速度上限。
// 作用：
// - 软件盲盒色布发车：识别板中心看到目标色布时持续发送 c。
// - 绿布消失后识别板恢复普通状态码，运行板解除此速度上限。
#ifndef BW_REMOTE_CLOTH_STOP_SPEED_CAP
#define BW_REMOTE_CLOTH_STOP_SPEED_CAP 0.01f
#endif
#pragma endregion

#pragma region 车辆属性参数
// 车辆属性（全局宏）
#define DISTANCE_FROM_VIEW_TO_CAR   (0.23f) // 图像(SET_IMAGE_CORE)到车轴距离（米）
#pragma endregion

#pragma region 图像特征量参数
// 特征量（全局宏）
// 角点
#define ANGLE_THRESHOLD_get_corners_conf_max  (120.0f) // 角点检测置信度上限（度）
#define ANGLE_THRESHOLD_get_corners_conf_min  (60.0f)  // 角点检测置信度下限（度）
// 曲线
#define CURVE_THRESHOLD                        (10.0f * PI32 / 180.0f) // 曲线判断角度阈值（弧度）
// 直线
#define ANGLE_THRESHOLD_is_straight            (8.0f) // 直线判断角度阈值（度）
#pragma endregion

#pragma region 元素判定参数
// 元素判定（全局宏）
// 双角点切换十字状态的 id 阈值
#define ID_THRESHOLD_crossing_state_change (80) // 十字判定角点索引阈值
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
// 十字 RUNNING 退出找线确认帧数
#define FRAME_THRESHOLD_crossing_running_to_none_found_line_counter (2) // RUNNING->NONE 找线确认帧
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

#endif /* _USER_COMMON_H_ */
