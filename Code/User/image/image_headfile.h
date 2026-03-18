#ifndef _SMARTCAR_IMAGE_HEADFILE_H_
#define _SMARTCAR_IMAGE_HEADFILE_H_

// 图像侧兼容聚合头：
// 作用：给历史代码或高层入口提供“一次性全包含”能力。
// 说明：核心实现文件应优先按最小必要 include 引入依赖，避免继续扩大该头的耦合范围。
#include "headfile.h"
#include "common.h"
#include "image_data.h"
#include "image_handle.h"
#include "image_process.h"
#include "image_math.h"
#include "transform_table.h"
#include "image_midline_process.h"
#include "element.h"
#include "element/circle.h"
#include "element/crossing.h"

#endif
