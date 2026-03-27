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
#include "LQ_TFT18_dri.hpp"
#include <stdlib.h>

static struct tft_spi_display tft18_spi;

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
void memset16(void *s, uint16_t c, size_t count)
{
    uint16_t *p = (uint16_t*)s;
    for (size_t i = 0; i < count; i++)
    {
        p[i] = __builtin_bswap16(c);
    }
}

/*!
 * @brief    TFT18初始化
 * @param    type ： 0:横屏  1：竖屏
 * @return   无
 * @note     如果修改管脚 需要修改初始化的管脚
 * @see      TFTSPI_Init(1);
 * @date     2025/4/28
 */
void TFTSPI_dri_init(uint8_t type)
{
    // 获取屏幕文件描述符
    tft18_spi.tft_fd = open("/dev/LQ_TFT_1.8", O_RDWR);
    if (tft18_spi.tft_fd < 0)
    {
        printf("Open file error\n");
        perror("[ERROR] Open file error");
        exit(1);
    }
    // 映射屏幕缓冲区
    tft18_spi.tft_fb = (uint16_t*)mmap(NULL, FB_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, tft18_spi.tft_fd, 0);
    if (tft18_spi.tft_fb == MAP_FAILED)
    {
        perror("mmap failed");
        close(tft18_spi.tft_fd);
        return;
    }
    // 选择横竖屏显示
    switch (type)
    {
        case 0:ioctl(tft18_spi.tft_fd, IOCTL_TFT_L_INIT);break;
        case 1:ioctl(tft18_spi.tft_fd, IOCTL_TFT_V_INIT);break;
        default:
            printf("TFTSPI_dri_init error\n");
            break;
    }
    struct Pixel {
        uint8_t tft18_w;
        uint8_t tft18_h; 
    };
    struct Pixel pix;
    ssize_t read_bytes = read(tft18_spi.tft_fd, &pix, sizeof(pix));
    if (read_bytes != sizeof(pix))
    {
        perror("read failed");
        close(tft18_spi.tft_fd);
        return;
    }
    tft18_spi.tft18_h = pix.tft18_h;
    tft18_spi.tft18_w = pix.tft18_w;
    // printf("type = %d, H = %d, W = %d\n", type, tft18_spi.tft18_h, tft18_spi.tft18_w);
}

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
void TFTSPI_dri_data_mod(uint8_t x, uint8_t y, uint16_t color)
{
    tft18_spi.tft_fb[y * tft18_spi.tft18_w + x] = __builtin_bswap16(color);
}

/*!
 * @brief    全屏显示单色画面
 * @param    color ：填充的颜色
 * @return   无
 * @note     起始、终止横坐标(0-127)，纵坐标(0-159),显示颜色uint16
 * @see      TFTSPI_dir_cls(u16YELLOW);
 * @date     2025/4/28
 */
void TFTSPI_dir_cls(uint16_t color_dat)
{
    memset16(tft18_spi.tft_fb, color_dat, SSIZE);
}

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
void TFTSPI_dri_fill_area(uint8_t xs, uint8_t ys, uint8_t xe, uint8_t ye, uint16_t color_dat)
{
    uint8_t i, j;
    for (j = 0; j < (ye - ys + 1); j++)
        for (i = 0; i < (xe - xs + 1); i++)
            TFTSPI_dri_data_mod(xs + i, ys + j, color_dat);
    ioctl(tft18_spi.tft_fd, IOCTL_TFT_FLUSH);
}

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
void TFTSPI_dri_draw_rectangle(uint8_t xs, uint8_t ys, uint8_t xe, uint8_t ye, uint16_t color_dat)
{
    TFTSPI_dri_draw_line(xs, ys, xs, ye, color_dat);    // 画矩形左边
    TFTSPI_dri_draw_line(xe, ys, xe, ye, color_dat);    // 画矩形右边
    TFTSPI_dri_draw_line(xs, ys, xe, ys, color_dat);    // 画矩形上边
    TFTSPI_dri_draw_line(xs, ye, xe, ye, color_dat);    // 画矩形下边
}

/*!
 * @brief   判断坐标是否超出规定范围
 * @param   coor : 坐标值
 * @param   min  : 限制最小值
 * @param   max  : 限制最大值
 * @return  超出返回 0，未超出返回 1
 */
uint8_t JudgeBeyondRange(uint8_t coor, uint8_t min, uint8_t max)
{
    if ((coor < min) || (coor > max - 1))
        return 0;
    return 1;
}

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
void TFTSPI_dri_draw_circle(uint8_t x, uint8_t y, uint8_t r, uint16_t color_dat)
{
    uint8_t dx, dy = r;
    if ((x < 0) || (x > tft18_spi.tft18_w - 1) || (y < 0) || (y > tft18_spi.tft18_h - 1))
        return;
    for (dx = 0; dx <= r; dx++)
    {
        while ((r * r + 1 - dx * dx) < (dy * dy))
            dy--;
        if (JudgeBeyondRange(x + dx, 0, tft18_spi.tft18_w) && JudgeBeyondRange(y - dy, 0, tft18_spi.tft18_h))
            TFTSPI_dri_data_mod(x + dx, y - dy, color_dat);
        if (JudgeBeyondRange(x - dx, 0, tft18_spi.tft18_w) && JudgeBeyondRange(y - dy, 0, tft18_spi.tft18_h))
            TFTSPI_dri_data_mod(x - dx, y - dy, color_dat);
        if (JudgeBeyondRange(x - dx, 0, tft18_spi.tft18_w) && JudgeBeyondRange(y + dy, 0, tft18_spi.tft18_h))
            TFTSPI_dri_data_mod(x - dx, y + dy, color_dat);
        if (JudgeBeyondRange(x + dx, 0, tft18_spi.tft18_w) && JudgeBeyondRange(y + dy, 0, tft18_spi.tft18_h))
            TFTSPI_dri_data_mod(x + dx, y + dy, color_dat);
        
        if (JudgeBeyondRange(x + dy, 0, tft18_spi.tft18_w) && JudgeBeyondRange(y - dx, 0, tft18_spi.tft18_h))
            TFTSPI_dri_data_mod(x + dy, y - dx, color_dat);
        if (JudgeBeyondRange(x - dy, 0, tft18_spi.tft18_w) && JudgeBeyondRange(y - dx, 0, tft18_spi.tft18_h))
            TFTSPI_dri_data_mod(x - dy, y - dx, color_dat);
        if (JudgeBeyondRange(x - dy, 0, tft18_spi.tft18_w) && JudgeBeyondRange(y + dx, 0, tft18_spi.tft18_h))
            TFTSPI_dri_data_mod(x - dy, y + dx, color_dat);
        if (JudgeBeyondRange(x + dy, 0, tft18_spi.tft18_w) && JudgeBeyondRange(y + dx, 0, tft18_spi.tft18_h))
            TFTSPI_dri_data_mod(x + dy, y + dx, color_dat);
    }
    ioctl(tft18_spi.tft_fd, IOCTL_TFT_FLUSH);
}

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
void TFTSPI_dri_draw_line(uint8_t xs, uint8_t ys, uint8_t xe, uint8_t ye, uint16_t color)
{
    int i, ds;
    int dx, dy, inc_x, inc_y;
    int xerr = 0, yerr = 0; // 初始化变量
    // 如果输入的坐标超出范围，则不会画线
    if ((xs < 0) || (xs > tft18_spi.tft18_w - 1) || (ys < 0) || (ys > tft18_spi.tft18_h - 1))
        return;
    if ((xe < 0) || (xe > tft18_spi.tft18_w - 1) || (ye < 0) || (ye > tft18_spi.tft18_h - 1))
        return;
    if (xs == xe)   // 如果是画直线则只需要对竖直坐标计数
    {
        for (i = 0; i < (ye - ys + 1); i++)
            TFTSPI_dri_data_mod(xs, ys + i, color);
    }
    else if (ys == ye)  // 如果是水平线则只需要对水平坐标计数
    {
        for (i = 0; i < (xe - xs + 1); i++)
            TFTSPI_dri_data_mod(xs + i, ys, color);
    }
    else    // 如果是斜线，则重新计算，使用画点函数画出直线
    {
        dx = xe - xs;   // 计算坐标增量
        dy = ye - ys;
        if (dx > 0)
            inc_x = 1;  // 设置单步方向
        else
        {
            inc_x = -1;
            dx = -dx;
        }
        if (dy > 0)
            inc_y = 1;  // 设置单步方向
        else
        {
            inc_y = -1;
            dy = -dy;
        }
        if (dx > dy)
            ds = dx;    // 选取基本增量坐标值
        else
            ds = dy;
        for (i = 0; i <= ds + 1; i++)   // 画线输出
        {
            TFTSPI_dri_data_mod(xs, ys, color);
            xerr += dx;
            yerr += dy;
            if (xerr > ds)
            {
                xerr -= ds;
                xs += inc_x;
            }
            if (yerr > ds)
            {
                yerr -= ds;
                ys += inc_y;
            }
        }
    }
    ioctl(tft18_spi.tft_fd, IOCTL_TFT_FLUSH);
}

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
void TFTSPI_dri_draw_dot(uint8_t x, uint8_t y, uint16_t color)
{
    TFTSPI_dri_data_mod(x, y, color);
    ioctl(tft18_spi.tft_fd, IOCTL_TFT_FLUSH);
}

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
void TFTSPI_dir_P6X8(uint8_t x, uint8_t y, uint8_t c_dat, uint16_t word_color, uint16_t back_color)
{
    uint8_t i, j;
    for (j = 0; j < 8; j++)
    {
        for (i = 0; i < 6; i++)
        {
            if ((Font_code8[c_dat - 32][i]) & (0x01 << j))
                TFTSPI_dri_data_mod(x * 6 + i, y * 8 + j, word_color);
            else
                TFTSPI_dri_data_mod(x * 6 + i, y * 8 + j, back_color);
        }
    }
}

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
void TFTSPI_dir_P8X8(uint8_t x, uint8_t y, uint8_t c_dat, uint16_t word_color, uint16_t back_color)
{
    uint8_t i, j;
    for (j = 0; j < 8; j++)
    {
        TFTSPI_dri_data_mod(x * 8, y * 8 + j, back_color);
        for (i = 0; i < 6; i++)
        {
            if ((Font_code8[c_dat - 32][i]) & (0x01 << j))
                TFTSPI_dri_data_mod(x * 8 + i + 1, y * 8 + j, word_color);
            else
                TFTSPI_dri_data_mod(x * 8 + i + 1, y * 8 + j, back_color);
        }
        TFTSPI_dri_data_mod(x * 8 + 7, y * 8 + j, back_color);
    }
}

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
void TFTSPI_dir_P8X16(uint8_t x, uint8_t y, uint8_t c_dat, uint16_t word_color, uint16_t back_color)
{
    uint8_t i, j;
    for (j = 0; j < 16; j++)
    {
        for (i = 0; i < 8; i++)
        {
            if ((Font_code16[c_dat - 32][j]) & (0x01 << i))
                TFTSPI_dri_data_mod(x * 8 + i, y * 16 + j, word_color);
            else
                TFTSPI_dri_data_mod(x * 8 + i, y * 16 + j, back_color);
        }
    }
}

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
void TFTSPI_dir_P6X8Str(uint8_t x, uint8_t y, const char *s_dat, uint16_t word_color, uint16_t back_color)
{
    while (*s_dat)
        TFTSPI_dir_P6X8(x++, y, *s_dat++, word_color, back_color);
    ioctl(tft18_spi.tft_fd, IOCTL_TFT_FLUSH);
}

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
void TFTSPI_dir_P8X8Str(uint8_t x, uint8_t y, const char *s_dat, uint16_t word_color, uint16_t back_color)
{
    while (*s_dat)
        TFTSPI_dir_P8X8(x++, y, *s_dat++, word_color, back_color);
    ioctl(tft18_spi.tft_fd, IOCTL_TFT_FLUSH);
}

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
void TFTSPI_dir_P8X16Str(uint8_t x, uint8_t y, const char *s_dat, uint16_t word_color, uint16_t back_color)
{
    while (*s_dat)
        TFTSPI_dir_P8X16(x++, y, *s_dat++, word_color, back_color);
    ioctl(tft18_spi.tft_fd, IOCTL_TFT_FLUSH);
}

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
void TFTSPI_dir_P16x16Str(uint8_t x, uint8_t y, const char *s_dat, uint16_t word_color, uint16_t back_color)
{
    uint8_t wm = 0, ii = 0, i, j;
    int adder = 1;
    while (s_dat[ii] != '\0')
    {
        wm = 0;
        adder = 1;
        while (hanzi_Idx[wm] > 127)
        {
            if (hanzi_Idx[wm] == (uint8_t)s_dat[ii])
            {
                if (hanzi_Idx[wm + 1] == s_dat[ii + 1])
                {
                    adder = wm * 16;
                    break;
                }
            }
            wm += 2;
        }

        if (adder != 1) // 显示汉字
        {
            for (j = 0; j < 32; j++)
            {
                for (i = 0; i < 8; i++)
                {
                    if ((hanzi16x16[adder]) & (0x80 >> i))
                    {
                        TFTSPI_dri_data_mod(x * 16 + i + (j % 2) * 8, y * 16 + (j / 2), word_color);
                    }
                    else
                    {
                        TFTSPI_dri_data_mod(x * 16 + i + (j % 2) * 8, y * 16 + (j / 2), back_color);
                    }
                }
                adder += 1;
            }
        }
        else // 显示空白字符
        {
        }
        // y+=1;//左右方向
        x += 1; // 上下方向
        ii += 2;
    }
    ioctl(tft18_spi.tft_fd, IOCTL_TFT_FLUSH);
}

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
void TFTSPI_dir_road(uint8_t wide_start, uint8_t high_start, uint8_t high, uint8_t wide, uint8_t *Pixle)
{
    uint64_t i, j;
    uint16_t color;
    for (j = 0; j < high; j++)
    {
        for (i = 0; i < wide; i++)
        {
            /* 将灰度转化为 RGB565 */
            color = (Pixle[j * wide + i] >> 3) << 11;
            color |= (Pixle[j * wide + i] >> 2) << 5;
            color |= Pixle[j * wide + i] >> 3;
            TFTSPI_dri_data_mod(wide_start + i, high_start + j, color);
        }
    }
    ioctl(tft18_spi.tft_fd, IOCTL_TFT_FLUSH);
}

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
void TFTSPI_dir_binRoad(uint8_t wide_start, uint8_t high_start, uint8_t high, uint8_t wide, uint8_t *Pixle)
{
    uint8_t i, j;
    /* 显示图像 */
    for (j = 0; j < high; j++)
    {
        for (i = 0; i < wide; i++)
        {
            if (Pixle[j * wide + i])
                TFTSPI_dri_data_mod(wide_start + i, high_start + j, u16WHITE); /* 显示 */
            else
                TFTSPI_dri_data_mod(wide_start + i, high_start + j, u16BLACK);
        }
    }
    TFTSPI_dir_flush();
}

/*!
 * @brief    TFT18屏 刷新屏幕
 * @param    无
 * @return   无
 * @note     无
 * @date     2025/4/28
 */
void TFTSPI_dir_flush()
{
    ioctl(tft18_spi.tft_fd, IOCTL_TFT_FLUSH);
}
