/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@编   写：龙邱科技
@邮   箱：chiusir@163.com
@编译IDE：Linux 环境、VSCode_1.93 及以上版本、Cmake_3.16 及以上版本
@使用平台：龙芯2K0300久久派和北京龙邱智能科技龙芯久久派拓展板
@相关信息参考下列地址
    网      站：http://www.lqist.cn
    淘 宝 店 铺：http://longqiu.taobao.com
    程序配套视频：https://space.bilibili.com/95313236
@软件版本：V1.0 版权所有，单位使用请先联系授权

@修改日期：2025-04-28
@修改内容：
@注意事项：
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
#include <stdio.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <stdint.h>
#include <sys/ioctl.h>
#include <LQ_Font.hpp>

/* TFT 参数定义 */
#define TFT18W        162
#define TFT18H        132

/* 颜色数据 */
#define u16RED		0xf800  // 红色
#define u16GREEN	0x07e0  // 绿色
#define u16BLUE	    0x001f  // 蓝色
#define u16PURPLE	0xf81f  // 紫色
#define u16YELLOW	0xffe0  // 黄色
#define u16CYAN	    0x07ff 	// 蓝绿色
#define u16ORANGE	0xfc08  // 橙色
#define u16BLACK	0x0000  // 黑色
#define u16WHITE	0xffff  // 白色

/* 帧缓冲大小 */
#define PageSize    16384
#define FB_SIZE     (PageSize * 3)

#define SSIZE (TFT18H * TFT18W)

#define IOCTL_TFT_FLUSH  _IO('t', 1)    // 刷新屏幕数据
#define IOCTL_TFT_L_INIT _IO('t', 2)    // 横屏初始化
#define IOCTL_TFT_V_INIT _IO('t', 3)    // 竖屏初始化

struct tft_spi_display {
    uint8_t tft18_w;    // 宽
    uint8_t tft18_h;    // 高
    int tft_fd;         // 文件描述符
    uint16_t *tft_fb;   // 映射空间
};

/*!
 * @brief   批量数据赋值
 * @param   s     : 需要赋值的空间头指针
 * @param   c     : 想要赋值的值(uint16_t类型)
 * @param   count : 想要赋值的数据量
 * @return  无
 * @note    如果修改管脚 需要修改初始化的管脚
 * @see     memset16(buf, u16RED, 10);
 * @date    2025/4/28
 */
void memset16(void *s, uint16_t c, size_t count);

/*!
 * @brief    TFT18初始化
 * @param    type ： 0:横屏  1：竖屏
 * @return   无
 * @note     如果修改管脚 需要修改初始化的管脚
 * @see      TFTSPI_Init(1);
 * @date     2025/4/28
 */
void TFTSPI_dri_init(uint8_t type);

/*!
 * @brief    修改指定坐标的数据
 * @param    x ：横坐标
 * @param    y ：纵坐标
 * @param    color ：颜色
 * @return   无
 * @note     起始、终止横坐标(0-127)，纵坐标(0-159),显示颜色uint16
 * @see      TFTSPI_dri_data_mod(10, 20, u16YELLOW);
 * @date     2025/4/28
 */
void TFTSPI_dri_data_mod(uint8_t x, uint8_t y, uint16_t color);

/*!
 * @brief    全屏显示单色画面
 * @param    color ：填充的颜色
 * @return   无
 * @note     起始、终止横坐标(0-127)，纵坐标(0-159),显示颜色uint16
 * @see      TFTSPI_dir_cls(u16YELLOW);
 * @date     2025/4/28
 */
void TFTSPI_dir_cls(uint16_t color_dat);

/*!
 * @brief    填充指定区域
 * @param    xs ：起始x
 * @param    ys ：起始y
 * @param    xe ：结束x
 * @param    ys ：结束y
 * @param    color ：填充的颜色
 * @return   无
 * @note     起始、终止横坐标(0-127)，纵坐标(0-159),显示颜色uint16
 * @see      TFTSPI_dri_fill_area(10, 20, 30, 40, u16YELLOW);
 * @date     2025/4/28
 */
void TFTSPI_dri_fill_area(uint8_t xs, uint8_t ys, uint8_t xe, uint8_t ye, uint16_t color_dat);

/*!
 * @brief    画矩形边框
 * @param    xs ：起始x
 * @param    ys ：起始y
 * @param    xe ：结束x
 * @param    ys ：结束y
 * @param    color_dat ：颜色
 * @return   无
 * @note     起始、终止横坐标(0-127)，纵坐标(0-159),显示颜色uint16
 * @see      TFTSPI_dri_draw_rectangle(10, 20, 30, 40, u16YELLOW);
 * @date     2025/4/28
 */
void TFTSPI_dri_draw_rectangle(uint8_t xs, uint8_t ys, uint8_t xe, uint8_t ye, uint16_t color_dat);

/*!
 * @brief   判断坐标是否超出规定范围
 * @param   coor : 坐标值
 * @param   min  : 限制最小值
 * @param   max  : 限制最大值
 * @return  超出返回 0，未超出返回 1
 */
uint8_t JudgeBeyondRange(uint8_t coor, uint8_t min, uint8_t max);

/*!
 * @brief    画圆
 * @param    x ：圆心x   (0-127)
 * @param    y ：圆心y   (0-159)
 * @param    r ：半径    (0-128)
 * @param    color_dat ：颜色
 * @return   无
 * @note     圆心坐标不要超出屏幕范围
 * @see      TFTSPI_dri_draw_circle(50, 50, 30, u16YELLOW);
 * @date     2025/4/28
 */
void TFTSPI_dri_draw_circle(uint8_t x, uint8_t y, uint8_t r, uint16_t color_dat);

/*!
 * @brief    画线
 * @param    xs ：起始x
 * @param    ys ：起始y
 * @param    xe ：结束x
 * @param    ys ：结束y
 * @param    color_dat ：颜色
 * @return   无
 * @note     起始、终止横坐标(0-127)，纵坐标(0-159),显示颜色uint16
 * @see      TFTSPI_dri_draw_line(10, 20, 30, 40, u16YELLOW);
 * @date     2025/4/28
 */
void TFTSPI_dri_draw_line(uint8_t xs, uint8_t ys, uint8_t xe, uint8_t ye, uint16_t color);

/*!
 * @brief    画点
 * @param    x ：x
 * @param    y ：y
 * @param    color_dat ：颜色
 * @return   无
 * @note     起始、终止横坐标(0-127)，纵坐标(0-159),显示颜色uint16
 * @see      TFTSPI_dri_draw_dot(10, 20, u16YELLOW);
 * @date     2025/4/28
 */
void TFTSPI_dri_draw_dot(uint8_t x, uint8_t y, uint16_t color);

/*!
 * @brief    液晶字符输出(6*8字体)
 * @param    x: 0 - 20	(行)
 * @param    y: 0 - 19	(列)
 * @param    word_color: 字体颜色
 * @param    back_color: 背景颜色
 * @return   无
 * @note     内部调用
 * @date     2025/4/28
 */
void TFTSPI_dir_P6X8(uint8_t x, uint8_t y, uint8_t c_dat, uint16_t word_color, uint16_t back_color);

/*!
 * @brief    液晶字符输出(8*8字体)
 * @param    x:0 - 15	(行)
 * @param    y:0 - 19	(列)
 * @param    word_color: 字体颜色
 * @param    back_color: 背景颜色
 * @return   无
 * @note     内部调用
 * @date     2025/4/28
 */
void TFTSPI_dir_P8X8(uint8_t x, uint8_t y, uint8_t c_dat, uint16_t word_color, uint16_t back_color);

/*!
 * @brief    液晶字符输出(8*16字体)
 * @param    x: 0 -15   (行)
 * @param    y: 0 -9  	 (列)
 * @param    word_color: 字体颜色
 * @param    back_color: 背景颜色
 * @return   无
 * @note     内部调用
 * @date     2025/4/28
 */
void TFTSPI_dir_P8X16(uint8_t x, uint8_t y, uint8_t c_dat, uint16_t word_color, uint16_t back_color);

/*!
 * @brief    液晶字符串输出(6*8字体)
 * @param    x: 0 - 20 (行)
 * @param    y: 0 - 19 (列)
 * @param    word_color: 字体颜色
 * @param    back_color: 背景颜色
 * @return   无
 * @note     无
 * @see      TFTSPI_dir_P6X8Str(1, 1, "123456", u16YELLOW, u16RED);
 * @date     2025/4/28
 */
void TFTSPI_dir_P6X8Str(uint8_t x, uint8_t y, const char *s_dat, uint16_t word_color, uint16_t back_color);

/*!
 * @brief    液晶字符串输出(8*8字体)
 * @param    x:0 - 15 (行)
 * @param    y:0 - 19 (列)
 * @param    word_color: 字体颜色
 * @param    back_color: 背景颜色
 * @return   无
 * @note     无
 * @see      TFTSPI_dir_P8X8Str(1, 1, "123456", u16YELLOW, u16RED);
 * @date     2025/4/28
 */
void TFTSPI_dir_P8X8Str(uint8_t x, uint8_t y, const char *s_dat, uint16_t word_color, uint16_t back_color);

/*!
 * @brief    液晶字符串输出(8*16字体)
 * @param    x: x: 0 -15   (行)
 * @param    y: y: 0 -9  	 (列)
 * @param    word_color: 字体颜色
 * @param    back_color: 背景颜色
 * @return   无
 * @note     无
 * @see      TFTSPI_dir_P8X16Str(1, 1, "123456", u16YELLOW, u16RED);
 * @date     2025/4/28
 */
void TFTSPI_dir_P8X16Str(uint8_t x, uint8_t y, const char *s_dat, uint16_t word_color, uint16_t back_color);

/*!
 * @brief    液晶汉字字符串输出(16*16字体)
 * @param    x: 0 - 7	(行)
 * @param    y: 0 - 9	(列)
 * @param    word_color: 字体颜色
 * @param    back_color: 背景颜色
 * @return   无
 * @note     汉字只能是字库里的 字库没有的需要自行添加
 * @see      TFTSPI_dir_P16x16Str(1, 1, "123456", u16YELLOW, u16RED);
 * @date     2025/4/28
 */
void TFTSPI_dir_P16x16Str(uint8_t x, uint8_t y, const char *s_dat, uint16_t word_color, uint16_t back_color);

/*!
 * @brief    TFT18屏 unsigned char 灰度数据显示
 * @param    high_start ： 显示图像开始位置
 * @param    wide_start ： 显示图像开始位置
 * @param    high ： 显示图像高度
 * @param    wide ： 显示图像宽度
 * @param    Pixle： 显示图像数据地址
 * @return   无
 * @note     注意 屏幕左上为 （0，0）
 * @see
 * @date     2025/4/28
 */
void TFTSPI_dir_road(uint8_t wide_start, uint8_t high_start, uint8_t high, uint8_t wide, uint8_t *Pixle);

/*!
 * @brief    TFT18屏 unsigned char 二值化数据显示
 * @param    high_start ： 显示图像开始位置
 * @param    wide_start ： 显示图像开始位置
 * @param    high ： 显示图像高度
 * @param    wide ： 显示图像宽度
 * @param    Pixle： 显示图像数据地址
 * @return   无
 * @note     注意 屏幕左上为 （0，0）
 * @see
 * @date     2025/4/28
 */
void TFTSPI_dir_binRoad(uint8_t wide_start, uint8_t high_start, uint8_t high, uint8_t wide, uint8_t *Pixle);

/*!
 * @brief    TFT18屏 刷新屏幕
 * @param    无
 * @return   无
 * @note     无
 * @date     2025/4/28
 */
void TFTSPI_dir_flush();
