#ifndef _USER_COMMON_H_
#define _USER_COMMON_H_

// 图像侧统一配置入口：
// - 所有图像算法阈值、编译期开关、默认值尽量集中在本文件。
// - 业务 .cc 文件应优先消费这里的语义化常量，避免继续散落局部宏。

#pragma region 图像基础参数
// 图像尺寸（全局宏，所有图像处理函数共享）
#define IMAGE_H               (120)   // 图像高度（像素）
#define IMAGE_W               (160)   // 图像宽度（像素）

#define PI32                  (3.1415926535898f) // 圆周率（float）

//绘图参数（全局宏）
#define DRAWRADIUS           (1)      // 绘图半径（像素）
#define BLACK_IN_GRAY        (0)      // 二值图黑色像素值
#define WHITE_IN_GRAY        (255)    // 二值图白色像素值

//物理参数（全局宏）
#define PIXPERMETER           (68.05f) // 逆透视后每米像素数
#define ROADWIDTH             (0.45f)  // 赛道宽度（米）

#define PT_MAXLEN             (80)    // 点列最大长度（左右线/中线/路径均使用）
#pragma endregion

#pragma region 图像预处理与寻线基础参数
#define SET_IMAGE_CORE_X   (80)  // 图像核心点X（像素）
#define SET_IMAGE_CORE_Y   (115) // 图像核心点Y（像素）

#define SEARCH_LINE_START_OFFSET  (20)   // 寻线起点横向偏移（像素）
#define SELFADAPT_KERNELSIZE  (7)        // 自适应滤波核尺寸（奇数）
#define FILTER_KERNELSIZE     (7)        // 边线滤波核尺寸（奇数）
#define SELFADAPT_OFFSET      (8)        // 自适应阈值偏移
#define RESAMPLEDIST          (0.02f)    // 重采样间距（米）
#define ANGLEDIST             (0.2f)     // 角度计算“跨度”（米）
#pragma endregion

#pragma region 中线融合基础参数
//中线（全局宏）
#define MIXED_LINE_DIFF_THRESHOLD_PIX    (0.1f)  // 左右中线混合差异阈值（像素）
#define MIXED_POINT_NUM_THRESHOLD       (5)      // 混合中线最少重合点数
#pragma endregion

#pragma region pure_angle预瞄与路径参数
// -------------------- pure_angle 预瞄与路径 --------------------
// 正常状态默认看 y=90 附近；前方弯越急，会动态把该值往更小处推，形成更强前瞻。
#define PUREANGLE_PREVIEW_BASE_IMAGE_Y   (90)
// pure_angle 预瞄图像行允许推到的最远位置（越小越看远，也越激进）。
#define PUREANGLE_PREVIEW_MIN_IMAGE_Y    (60)
// 预瞄曲率链的局部曲率计算跨度（点数）。
#define PUREANGLE_PREVIEW_CURV_DIST      (3)
// 预瞄曲率链的曲率 NMS 窗口大小（点数）。
#define PUREANGLE_PREVIEW_CURV_NMS_KERNEL (5)
// 曲率阈值下限（1-cos(theta) 域）：
// 约等于 theta≈10deg，对应较轻微弯道，不触发明显前推。
#define PUREANGLE_PREVIEW_CURVE_LOW      (0.0152f)
// 曲率阈值上限（1-cos(theta) 域）：
// 约等于 theta≈35deg，对应明显弯道，前推到最大。
#define PUREANGLE_PREVIEW_CURVE_HIGH     (0.1809f)
// 纯局部几何连续映射所允许的最大前推量（图像行）。
#define PUREANGLE_PREVIEW_SHIFT_MAX      (20)
// 特殊几何形态下的附加前推量：连续弯通常比普通弯更需要提前切入。
#define PUREANGLE_PREVIEW_S_CURVE_SHIFT  (12)
// 直角弯/大角度弯下的附加前推量。
#define PUREANGLE_PREVIEW_ANGLE_SHIFT    (18)
// 环岛或长圆弧下的附加前推量。
#define PUREANGLE_PREVIEW_ROUND_SHIFT    (10)
// 中线几何分类里的“最小环岛半径”阈值（逆透视像素）：
// 只用于 pure_angle 动态预瞄的粗粒度赛道判别。
#define MID_TRACK_ROUNDABOUT_MIN_RADIUS_PIX (10.0f)
// 中线几何分类里的“S 弯最小弧段长度”阈值（逆透视像素）：
// 太短的弧段通常只是噪声分段，不参与 S 弯判定。
#define MID_TRACK_S_CURVE_MIN_ARC_LEN_PIX  (14.0f)
// 用于“路径并轨到中线”的参考图像行（越小越看远，越大越看近）
#define PATH_BLEND_REF_IMAGE_Y           (80)
#pragma endregion

#pragma region pure_angle预瞄过渡参数
// pure_angle 预瞄过渡总开关：
// 作用位置：image_handle.cc 的 preview_img_y 平滑过渡链。
// 作用：限制动态预瞄点帧间跳变，避免 pure_angle 因预瞄目标切换而突变。
// 调大/关闭：更跟手，但更容易抖动。
// 调小/开启：更平滑，但前瞻切换会更慢。
#ifndef PUREANGLE_PREVIEW_TRANSITION_ENABLE
#define PUREANGLE_PREVIEW_TRANSITION_ENABLE 1
#endif

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
#pragma endregion

#pragma region pure_angle丢线补偿参数
// 丢线趋势外推总开关：
// 使用位置：image_process.cc / pure_angle_apply_lost_strategy。
// 作用：在短时双边丢线或元素阶段缺测时，允许沿上一时刻趋势外推 pure_angle。
// 关闭后：丢线时更保守，但急弯/元素阶段更容易“掉头感”。
#ifndef PUREANGLE_LOST_TREND_ENABLE
#define PUREANGLE_LOST_TREND_ENABLE 1
#endif

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
// pure_angle 趋势前馈总开关：
// 使用位置：image_process.cc / pure_angle_apply_pre_control。
// 作用：当转向角还在同方向持续增大时，额外补一点前馈量。
// 关闭：行为更保守，更依赖原始 pure_angle。
#ifndef PUREANGLE_PRE_CTRL_ENABLE
#define PUREANGLE_PRE_CTRL_ENABLE 0
#endif

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
// 斑马线横向搜索步长（像素）：
// 使用位置：element/zebra.cc。
// 作用：决定 run-length 检测沿横向的采样跨度。
// 调大：计算更快，但细节更少。
// 调小：更细致，但更易受局部噪声影响。
#ifndef ZEBRA_STEP
#define ZEBRA_STEP 50
#endif

// 历史保留阈值：连续黑点判定下限。
// 使用位置：旧斑马线逻辑兼容；当前主逻辑已不依赖它做最终判定。
#ifndef ZEBRA_CONSECUTIVE
#define ZEBRA_CONSECUTIVE 3
#endif

// 近距离检测长度（点数）：
// 使用位置：element/zebra.cc。
// 作用：限制斑马线在近车区域内的检测窗口。
#ifndef NEAR_DETECT_MAX
#define NEAR_DETECT_MAX (30)
#endif

// 至少需要这么多段黑白色块，才认为有“斑马线形态”。
// 使用位置：element/zebra.cc。
// 调大：更严格，误检减少。
// 调小：更灵敏，但更容易把普通纹理误判为斑马线。
#ifndef ZEBRA_MIN_RUNS
#define ZEBRA_MIN_RUNS (6)
#endif

// 斑马线等距离判定开关：
// 使用位置：element/zebra.cc。
// 作用：控制是否启用“白块/黑块长度接近且内部离散较小”这一层规则性筛选。
// 1：启用等距离判定，误检更少，但对破损/透视畸变更敏感。
// 0：关闭等距离判定，只保留基本的黑白色块数量判定，检测更宽松。
// 推荐：赛道斑马线规则、拍摄稳定时保持开启；若现场纹理或透视导致经常漏检可临时关闭。
#ifndef BW_ENABLE_ZEBRA_RUN_LENGTH_CHECK
#define BW_ENABLE_ZEBRA_RUN_LENGTH_CHECK 0
#endif

// 白块均值与黑块均值的允许相对差值：
// 使用位置：element/zebra.cc。
// 作用：约束白黑色块的平均长度要接近。
// 调大：更宽松。
// 调小：更严格。
#ifndef ZEBRA_RUN_DIFF_RATIO
#define ZEBRA_RUN_DIFF_RATIO (0.35f)
#endif

// 单个色块相对同色均值的允许离散比例：
// 使用位置：element/zebra.cc。
// 作用：要求色块长度内部也要比较整齐。
// 调大：更宽松。
// 调小：更严格，抗误检更强。
#ifndef ZEBRA_RUN_SPREAD_RATIO
#define ZEBRA_RUN_SPREAD_RATIO (0.45f)
#endif

// 斑马线解锁冷却时间（毫秒）：
// 使用位置：image_process.cc。
// 作用：手动解除 zebra_stop 后，短时间内不允许再次触发。
#ifndef ZEBRA_COOLDOWN_MS
#define ZEBRA_COOLDOWN_MS 1200
#endif

// 检测到斑马线后延迟停车的时间（毫秒）：
// 使用位置：image_process.cc。
// 作用：在触发后先走一小段，再进入锁停。
#ifndef ZEBRA_STOP_DELAY_MS
#define ZEBRA_STOP_DELAY_MS 3000
#endif

// 历史保留：曾用特殊 pure_angle 表达停车。
// 当前不再使用，仅保留兼容外部旧引用。
#ifndef ZEBRA_STOP_ANGLE
#define ZEBRA_STOP_ANGLE 111.11f
#endif
#pragma endregion

#pragma region 识别链与图传默认参数
// -------------------- 识别链 / 图传链默认开关 --------------------
// 模型识别链默认开关：
// 使用位置：recognition_chain.cc / DefaultEnabled。
// 作用：控制整条识别链默认是否可用；运行时仍可被命令行覆盖。
#ifndef BW_ENABLE_RECOGNITION
#define BW_ENABLE_RECOGNITION 0
#endif

// 识别触发带中心 y：
// 使用位置：recognition_chain.cc / TryEnterRecognition。
// 坐标系：普通态触发帧 160x120。
// 作用：只有红框中心 y 落在该位置附近时，才允许进入识别态。
// 调大：要求目标更靠近画面下方才触发。
// 调小：目标更靠近画面上方也可触发。
#ifndef BW_RECOG_TRIGGER_CENTER_Y
#define BW_RECOG_TRIGGER_CENTER_Y 80
#endif

// 识别触发带 y 容差：
// 使用位置：recognition_chain.cc / TryEnterRecognition。
// 作用：定义“y≈BW_RECOG_TRIGGER_CENTER_Y”的允许范围。
// 例如中心 y=80、容差 10，对应允许区间 [70, 90]。
// 调大：更宽松，更容易触发。
// 调小：更严格，更依赖目标刚好经过指定横带。
#ifndef BW_RECOG_TRIGGER_CENTER_Y_TOL
#define BW_RECOG_TRIGGER_CENTER_Y_TOL 10
#endif

// 识别触发逆透视长方形判定开关：
// 使用位置：recognition_chain.cc / TryEnterRecognition。
// 作用：控制是否要求“红框在逆透视后仍然是横向长方形”。
// 1：启用该判定，只有上/下边为长边时才允许进入识别态。
// 0：关闭该判定，退回为“红框 + y≈80”旧逻辑。
#ifndef BW_RECOG_TRIGGER_IPM_RECT_ENABLE
#define BW_RECOG_TRIGGER_IPM_RECT_ENABLE 1
#endif

// 识别触发逆透视宽高比阈值：
// 使用位置：recognition_chain.cc / red_rect_is_horizontal_ipm_rect。
// 作用：要求逆透视后 top/bottom 边长度至少达到 left/right 边长度的该倍数。
// 调大：更严格，更偏向只接受明显横向长方形。
// 调小：更宽松，近似方形也更容易通过。
// 推荐：先从 1.30 起调，若漏触发可适当下调，若误触发可适当上调。
#ifndef BW_RECOG_TRIGGER_IPM_MIN_WIDTH_HEIGHT_RATIO
#define BW_RECOG_TRIGGER_IPM_MIN_WIDTH_HEIGHT_RATIO 1.30f
#endif

// 识别结果动作链默认开关：
// 使用位置：recognition_chain.cc / recognition_action_enabled。
// 作用：允许“只识别、不改跟线策略”。
#ifndef BW_ENABLE_RECOGNITION_ACTION
#define BW_ENABLE_RECOGNITION_ACTION 0
#endif
#pragma endregion

#pragma region 图传开关与图传模式切换
// 图传链默认开关：
// 使用位置：stream_chain.cc / DefaultEnabled。
// 作用：控制图传服务器默认是否启动；运行时仍可被命令行覆盖。
#ifndef BW_ENABLE_STREAM
#define BW_ENABLE_STREAM 1
#endif

// 普通巡线分支图传底图模式：
// 使用位置：vision_runtime.cc / RenderLineTrackingView。
// 1：图传显示“二值图转 BGR 后叠加轨迹”，更利于看巡线处理结果。
// 0：图传显示“低分辨率彩图叠加轨迹”，更利于看现场原始画面。
// 说明：该开关只影响普通巡线分支；识别态和绕行态仍保持彩图显示。
#ifndef BW_STREAM_LINE_USE_BINARY_VIEW
#define BW_STREAM_LINE_USE_BINARY_VIEW 0
#endif
#pragma endregion

#pragma region 历史保留切换参数
#define FRAMENONE             (3) // 预留：状态保持帧数（未使用/保留）
#define FRAMETOLEFT           (5) // 预留：左切换保护帧（未使用/保留）
#define FRAMETORIGHT          (5) // 预留：右切换保护帧（未使用/保留）
#pragma endregion

#pragma region 车辆属性参数
//车辆属性（全局宏）
#define DISTANCE_FROM_VIEW_TO_CAR   (0.23f)   // 图像(SET_IMAGE_CORE)到车轴距离（米）
#define WIDTH_OF_CAR                (0.18f)   // 车体宽度（米）
#pragma endregion

#pragma region 图像特征量参数
//特征量（全局宏）
//角点
#define ANGLE_THRESHOLD_get_corners_conf_max             (120.0f)   // 角点检测置信度上限（度）
#define ANGLE_THRESHOLD_get_corners_conf_min             (60.0f)    // 角点检测置信度下限（度）
#define ID_THRESHOLD_get_corners_near_detect             (15)       // 角点近距离索引上限
//曲线
#define CURVE_THRESHOLD       (10.0f * PI32 / 180.0f)      // 曲线判断角度阈值（弧度）
//直线
#define ANGLE_THRESHOLD_is_straight         (8.0f)         // 直线判断角度阈值（度）
#pragma endregion

#pragma region 左右线切换保留参数
//左右线切换点数差阈值（全局宏）
#define PTS_THRESHOLD_follow_left               (0) // 左跟线点数差阈值（保留）
#pragma endregion

#pragma region 元素判定参数
//元素判定（全局宏）
//双角点切换十字状态的id阈值
#define ID_THRESHOLD_crossing_state_change      (20) // 十字判定角点索引阈值
//环岛保护帧数
#define FRAME_THRESHOLD_roundabout_protect_frame        (3) // 环岛状态保护帧
//环岛或斜入十字判定帧数
#define FRAME_THRESHOLD_roundabout_or_crossing_frame (3) // 元素投票确认帧
//斜入十字保护帧数
#define FRAME_THRESHOLD_one_corner_crossing_protect_frame (3) // 十字保护帧
//角点对侧直线性窗口判断阈值
#define WINDOW_THRESHOLD_roundabout_opposite_straightness    (10) // 对侧直线性窗口半宽（点数）
#pragma endregion

#pragma region 元素状态机帧阈值参数
//元素帧数阈值（全局宏）
//十字路口丢线帧数阈值
#define FRAME_THRESHOLD_crossing_lost_line_counter       (2)    // 十字入内阶段允许的连续丢线帧
//环岛入环丢线阈值
#define FRAME_THRESHOLD_roundabout_begin_to_in_lost_line_counter       (2) // BEGIN->IN 丢线阈值
//环岛入环重新找线阈值
#define FRAME_THRESHOLD_roundabout_begin_to_in_found_line_counter       (2) // BEGIN->IN 找线阈值
//环岛出环岛角点连续判定阈值
#define FRAME_THRESHOLD_roundabout_running_to_out_corner_counter       (0) // RUNNING->OUT 角点连续帧
//环岛结束丢线阈值
#define FRAME_THRESHOLD_roundabout_end_lost_line_counter       (2)    // END 退出丢线阈值
//环岛结束重新找线阈值
#define FRAME_THRESHOLD_roundabout_end_found_line_counter       (2)    // END 退出找线阈值
#pragma endregion

#pragma region 赛道类型角度阈值参数
//赛道类型角度阈值（全局宏）
//曲线判断角度
#define ANGLE_THRESHOLD_track_type_curve            (15.0f ) // 曲线判定阈值（度）
//急弯判断角度
#define ANGLE_THRESHOLD_track_type_sharp_curve      (35.0f ) // 急弯判定阈值（度）
//急弯保护
#define FRAME_THRESHOLD_sharp_curve_protect_frame   ( 25 )  // 急弯保护帧
#pragma endregion

#endif /* USER_CAMERA_COMMON_H_ */
