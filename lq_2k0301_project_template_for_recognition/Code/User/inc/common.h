#ifndef _USER_COMMON_H_
#define _USER_COMMON_H_

// 识别板图像侧统一配置入口：
// - 当前只保留识别链和图传链真实使用的编译期参数。
// - 旧单板巡线/元素/控制参数已从识别板工程中剥离。

#pragma region 识别板功能开关
// 识别链编译期默认开关：
// 运行时仍可通过 --recognition / --no-recognition 覆盖。
#ifndef BW_ENABLE_RECOGNITION
#define BW_ENABLE_RECOGNITION 0
#endif

// 图传编译期默认开关：
// 运行时仍可通过 --stream / --no-stream 覆盖。
#ifndef BW_ENABLE_STREAM
#define BW_ENABLE_STREAM 1
#endif
#pragma endregion

#pragma region 双板通信参数
// 纯事件流模式下，同一个新事件重复发送的帧数。
// 作用：在不加心跳的前提下，提高运行板稳定收到同一事件的概率。
#ifndef BW_BOARD_EVENT_REPEAT_FRAMES
#define BW_BOARD_EVENT_REPEAT_FRAMES 8
#endif
#pragma endregion

#pragma region 红框触发几何参数
// 红框中心 y 目标行：
// 识别板现在直接按 640x480 原图做触发几何。
#ifndef BW_RECOG_TRIGGER_CENTER_Y
#define BW_RECOG_TRIGGER_CENTER_Y 320
#endif

// 红框中心 y 容差：
// 例如 center_y=320、tol=40，则允许区间为 [280, 360]。
#ifndef BW_RECOG_TRIGGER_CENTER_Y_TOL
#define BW_RECOG_TRIGGER_CENTER_Y_TOL 40
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
