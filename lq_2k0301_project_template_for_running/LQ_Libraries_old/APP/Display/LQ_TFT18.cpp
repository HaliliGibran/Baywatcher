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
@参考项目链接：https://github.com/AirFortressIlikara/ls2k0300_peripheral_library

@修改日期：2025-03-04
@修改内容：
@注意事项：
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
#include "LQ_TFT18.hpp"

/* 
    注意:如果使用硬件SPI的方式，拓展板对应使用的是SPI1端口，需要在设备树中配置好SPI1才可以使用。
         并且需要提前查看SPI1对应设备文件，与端口初始化处文件名称是否一致
 */
#define USE_QSPI 0 // 0为使用软件SPI的方式，1为使用硬件SPI的方式

/*!
 * @brief    毫秒延时函数
 * @param    无
 * @return   无
 * @note     无
 * @see      lq_tft_delayms(100);
 */
static void lq_tft_delayms(u16 ms)
{
    usleep(ms * 1000);
}

//-----端口初始化----//
HWGpio TFTSPI_DC_fd;//(TFTSPI_DC, GPIO_Mode_Out);
HWGpio TFTSPI_RST_fd;//(TFTSPI_RST, GPIO_Mode_Out);
#if USE_QSPI
    HW_SPI TFT_HW_SPI("/dev/spidev1.0");
#else
    HWGpio TFTSPI_CS_fd;//(TFTSPI_CS, GPIO_Mode_Out);
    HWGpio TFTSPI_SCK_fd;//(TFTSPI_SCK, GPIO_Mode_Out);
    HWGpio TFTSPI_SDI_fd;//(TFTSPI_SDI, GPIO_Mode_Out);
#endif

/*!
 * @brief    TFT18初始化
 * @param    type ： 0:横屏  1：竖屏
 * @return   无
 * @note     如果修改管脚 需要修改初始化的管脚
 * @see      TFTSPI_Init(1);
 * @date     2019/6/13 星期四
 */
void TFTSPI_Init(u8 type)
{
    TFTSPI_RST_L;
    lq_tft_delayms(50);
    TFTSPI_RST_H;
    lq_tft_delayms(50);
    TFTSPI_Write_Cmd(0x11); // 关闭睡眠，振荡器工作
    lq_tft_delayms(50);
    TFTSPI_Write_Cmd(0x3a); // 每次传送16位数据(VIPF3-0=0101)，每个像素16位(IFPF2-0=101)
    TFTSPI_Write_Byte(0x55);
    TFTSPI_Write_Cmd(0x26);
    TFTSPI_Write_Byte(0x04);
    TFTSPI_Write_Cmd(0xf2); // Driver Output Control(1)
    TFTSPI_Write_Byte(0x01);
    TFTSPI_Write_Cmd(0xe0); // Driver Output Control(1)
    TFTSPI_Write_Byte(0x3f);
    TFTSPI_Write_Byte(0x25);
    TFTSPI_Write_Byte(0x1c);
    TFTSPI_Write_Byte(0x1e);
    TFTSPI_Write_Byte(0x20);
    TFTSPI_Write_Byte(0x12);
    TFTSPI_Write_Byte(0x2a);
    TFTSPI_Write_Byte(0x90);
    TFTSPI_Write_Byte(0x24);
    TFTSPI_Write_Byte(0x11);
    TFTSPI_Write_Byte(0x00);
    TFTSPI_Write_Byte(0x00);
    TFTSPI_Write_Byte(0x00);
    TFTSPI_Write_Byte(0x00);
    TFTSPI_Write_Byte(0x00);
    TFTSPI_Write_Cmd(0xe1); // Driver Output Control(1)
    TFTSPI_Write_Byte(0x20);
    TFTSPI_Write_Byte(0x20);
    TFTSPI_Write_Byte(0x20);
    TFTSPI_Write_Byte(0x20);
    TFTSPI_Write_Byte(0x05);
    TFTSPI_Write_Byte(0x00);
    TFTSPI_Write_Byte(0x15);
    TFTSPI_Write_Byte(0xa7);
    TFTSPI_Write_Byte(0x3d);
    TFTSPI_Write_Byte(0x18);
    TFTSPI_Write_Byte(0x25);
    TFTSPI_Write_Byte(0x2a);
    TFTSPI_Write_Byte(0x2b);
    TFTSPI_Write_Byte(0x2b);
    TFTSPI_Write_Byte(0x3a);
    TFTSPI_Write_Cmd(0xb1);  // 0xb1      	//设置屏幕刷新频率
    TFTSPI_Write_Byte(0x00); // 0x08		//DIVA=8
    TFTSPI_Write_Byte(0x00); // 0x08		//VPA =8，约90Hz
    TFTSPI_Write_Cmd(0xb4);  // LCD Driveing control
    TFTSPI_Write_Byte(0x07); // NLA=1,NLB=1,NLC=1
    TFTSPI_Write_Cmd(0xc0);  // LCD Driveing control  Power_Control1
    TFTSPI_Write_Byte(0x0a);
    TFTSPI_Write_Byte(0x02);
    TFTSPI_Write_Cmd(0xc1); // LCD Driveing control
    TFTSPI_Write_Byte(0x02);
    TFTSPI_Write_Cmd(0xc5); // LCD Driveing control
    TFTSPI_Write_Byte(0x4f);
    TFTSPI_Write_Byte(0x5a);
    TFTSPI_Write_Cmd(0xc7); // LCD Driveing control
    TFTSPI_Write_Byte(0x40);
    TFTSPI_Write_Cmd(0x2a);  // 配置MCU可操作的LCD内部RAM横坐标起始、结束参数
    TFTSPI_Write_Byte(0x00); // 横坐标起始地址0x0000
    TFTSPI_Write_Byte(0x00);
    TFTSPI_Write_Byte(0x00); // 横坐标结束地址0x007f(127)
    TFTSPI_Write_Byte(0xa8); // 7f
    TFTSPI_Write_Cmd(0x2b);  // 配置MCU可操作的LCD内部RAM纵坐标起始结束参数
    TFTSPI_Write_Byte(0x00); // 纵坐标起始地址0x0000
    TFTSPI_Write_Byte(0x00);
    TFTSPI_Write_Byte(0x00); // 纵坐标结束地址0x009f(159)
    TFTSPI_Write_Byte(0xb3); // 9f
    TFTSPI_Write_Cmd(0x36);  // 配置MPU和DDRAM对应关系
    if (type)
        TFTSPI_Write_Byte(0xC0); // 竖屏显示          //MX=1,MY=1
    else
        TFTSPI_Write_Byte(0xA0); // 横屏显示

    TFTSPI_Write_Cmd(0xb7);  // LCD Driveing control
    TFTSPI_Write_Byte(0x00); // CRL=0
    TFTSPI_Write_Cmd(0x29);  // 开启屏幕显示
    TFTSPI_Write_Cmd(0x2c);  // 设置为LCD接收数据/命令模式
}

/*!
 * @brief    简单延时函数
 * @param    Del ：延时时间
 * @return   无
 * @note     内部调用
 * @see      tft18delay_1us(1);
 * @date     2025/2/23
 */
void tft18delay_1us(u32 Del)
{
    usleep(Del);
}

#if USE_QSPI

/*!
 * @brief    写命令
 * @param    cmd ：命令
 * @return   无
 * @note     内部调用
 * @see      TFTSPI_Write_Cmd(0xb7); //LCD Driveing control
 * @date     2025/2/23
 */
void TFTSPI_Write_Cmd(u8 cmd)
{
    TFTSPI_DC_L; // A0=0
    TFT_HW_SPI.SPI_Write(&cmd, sizeof(cmd));
}

/*!
 * @brief    写字节
 * @param    dat ：数据
 * @return   无
 * @note     内部调用
 * @see      TFTSPI_Write_Byte(0x00);	//CRL=0
 * @date     2025/2/23
 */
void TFTSPI_Write_Byte(u8 dat)
{
    TFTSPI_DC_H; // A0=1 发送数据
    TFT_HW_SPI.SPI_Write(&dat, sizeof(dat));
}

/*!
 * @brief    写半字
 * @param    dat ：数据
 * @return   无
 * @note     无
 * @see      TFTSPI_Write_Word(0xFFFF);
 * @date     2025/2/23
 */
void TFTSPI_Write_Word(u16 dat)
{
    TFTSPI_DC_H; // A0=1
    uint8_t dat1 = dat >> 8;
    uint8_t dat2 = dat;
    TFT_HW_SPI.SPI_Write(&dat1, sizeof(uint8_t));
    TFT_HW_SPI.SPI_Write(&dat2, sizeof(uint8_t));
}

#else

/*!
 * @brief    写命令
 * @param    cmd ：命令
 * @return   无
 * @note     内部调用
 * @see      TFTSPI_Write_Cmd(0xb7); //LCD Driveing control
 * @date     2019/6/13 星期四
 */
void TFTSPI_Write_Cmd(u8 cmd)
{
    u8 i;

    TFTSPI_DC_L; // A0=0发送命令
    for (i = 0; i < 8; i++)
    {
        TFTSPI_SCK_L; // SCK=0
        if (cmd & 0x80)
        {
            TFTSPI_SDI_H; // SDI=1
        }
        else
        {
            TFTSPI_SDI_L; // SDI=0
        }
        TFTSPI_SCK_H; // SCK=1
        cmd = (cmd << 1);
    }
}

/*!
 * @brief    写字节
 * @param    dat ：数据
 * @return   无
 * @note     内部调用
 * @see      TFTSPI_Write_Byte(0x00);	//CRL=0
 * @date     2019/6/13 星期四
 */
void TFTSPI_Write_Byte(u8 dat)
{
    u8 i;

    TFTSPI_DC_H; // A0=1发送数据
    for (i = 0; i < 8; i++)
    {
        TFTSPI_SCK_L; // SCK=0
        if (dat & 0x80)
        {
            TFTSPI_SDI_H;
        } // SDI=1
        else
        {
            TFTSPI_SDI_L;
        } // SDI=0
        TFTSPI_SCK_H; // SCK=1
        dat = (dat << 1);
    }
}

/*!
 * @brief    写半字
 * @param    dat ：数据
 * @return   无
 * @note     无
 * @see      TFTSPI_Write_Word(0xFFFF);
 * @date     2019/6/13 星期四
 */
void TFTSPI_Write_Word(u16 dat)
{
    u8 i;

    TFTSPI_DC_H; // A0=1
    for (i = 0; i < 16; i++)
    {
        TFTSPI_SCK_L; // SCK=0
        if (dat & 0x8000)
        {
            TFTSPI_SDI_H;
        } // SDI=1
        else
        {
            TFTSPI_SDI_L;
        } // SDI=0
        TFTSPI_SCK_H; // SCK=1
        dat <<= 1;
    }
}

#endif

/*!
 * @brief    重新定位输入信息位置
 * @param    xs ：起始x
 * @param    ys ：起始y
 * @param    xe ：结束x
 * @param    ys ：结束y
 * @return   无
 * @note     内部调用
 * @see      TFTSPI_Set_Pos(10, 20, 30, 40);
 * @date     2019/6/13 星期四
 */
void TFTSPI_Set_Pos(u8 xs, u8 ys, u8 xe, u8 ye)
{
    TFTSPI_Write_Cmd(0x2A);  // Colulm addRSTs set
    TFTSPI_Write_Byte(0x00); // 行起始坐标高8位，始终为零
    TFTSPI_Write_Byte(xs);   // 行起始坐标低8位
    TFTSPI_Write_Byte(0x00); // 行终止坐标高8位，始终为零
    TFTSPI_Write_Byte(xe);   // 行终止坐标低8位
    TFTSPI_Write_Cmd(0x2B);  // Colulm addRSTs set
    TFTSPI_Write_Byte(0x00); // 列起始坐标高8位，始终为零
    TFTSPI_Write_Byte(ys);   // 列起始坐标低8位
    TFTSPI_Write_Byte(0x00); // 列终止坐标高8位，始终为零
    TFTSPI_Write_Byte(ye);   // 列终止坐标低8位
    TFTSPI_Write_Cmd(0x2C);  // GRAM接收MCU数据或命令
}

/*!
 * @brief    重置地址
 * @param    无
 * @return   无
 * @note     内部调用
 * @see      TFTSPI_Addr_Rst();
 * @date     2019/6/13 星期四
 */
void TFTSPI_Addr_Rst(void)
{
    TFTSPI_Write_Cmd(0x2a);  // 配置MCU可操作的LCD内部RAM横坐标起始、结束参数
    TFTSPI_Write_Byte(0x00); // 横坐标起始地址0x0000
    TFTSPI_Write_Byte(0x00);
    TFTSPI_Write_Byte(0x00); // 横坐标结束地址0x007f(127)
    TFTSPI_Write_Byte(0xa8); // 7f
    TFTSPI_Write_Cmd(0x2b);  // 配置MCU可操作的LCD内部RAM纵坐标起始结束参数
    TFTSPI_Write_Byte(0x00); // 纵坐标起始地址0x0000
    TFTSPI_Write_Byte(0x00);
    TFTSPI_Write_Byte(0x00); // 纵坐标结束地址0x009f(159)
    TFTSPI_Write_Byte(0xb3); // 9f
    TFTSPI_Write_Cmd(0x2C);  // GRAM接收MCU数据或命令
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
 * @see      TFTSPI_Fill_Area(10, 20, 30, 40, u16YELLOW);
 * @date     2019/6/13 星期四
 */
void TFTSPI_Fill_Area(u8 xs, u8 ys, u8 xe, u8 ye, u16 color)
{
    u32 i, j;

    TFTSPI_Set_Pos(xs, ys, xe, ye);
    for (i = 0; i < (xe - xs); i++)
    {
        for (j = 0; j < (ye - ys); j++)
            TFTSPI_Write_Word(color);
    }
}

/*!
 * @brief    全屏显示单色画面
 * @param    color ：填充的颜色
 * @return   无
 * @note     起始、终止横坐标(0-127)，纵坐标(0-159),显示颜色uint16
 * @see      TFTSPI_CLS(u16YELLOW);
 * @date     2019/6/13 星期四
 */
void TFTSPI_CLS(u16 color)
{
    u32 i, j;

    TFTSPI_Addr_Rst();

    for (i = 0; i < TFT18W; i++) // 160
    {
        for (j = 0; j < TFT18H; j++)
            TFTSPI_Write_Word(color);
    }
}

/*!
 * @brief    填充矩形区域
 * @param    xs ：起始x
 * @param    ys ：起始y
 * @param    xe ：结束x
 * @param    ys ：结束y
 * @param    color_dat ：填充的颜色
 * @return   无
 * @note     起始、终止横坐标(0-127)，纵坐标(0-159),显示颜色uint16
 * @see      TFTSPI_Draw_Part(10, 20, 30, 40, u16YELLOW);
 * @date     2019/6/13 星期四
 */
void TFTSPI_Draw_Part(u8 xs, u8 ys, u8 xe, u8 ye, u16 color_dat)
{
    u8 i, j;

    TFTSPI_Set_Pos(xs, ys, xe, ye);
    for (j = 0; j < (ye - ys + 1); j++)
    {
        for (i = 0; i < (xe - xs + 1); i++)
            TFTSPI_Write_Word(color_dat);
    }
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
 * @see      TFTSPI_Draw_Rectangle(10, 20, 30, 40, u16YELLOW);
 * @date     2019/6/13 星期四
 */
void TFTSPI_Draw_Rectangle(u8 xs, u8 ys, u8 xe, u8 ye, u16 color_dat)
{
    TFTSPI_Draw_Line(xs, ys, xs, ye, color_dat); // 画矩形左边
    TFTSPI_Draw_Line(xe, ys, xe, ye, color_dat); // 画矩形右边
    TFTSPI_Draw_Line(xs, ys, xe, ys, color_dat); // 画矩形上边
    TFTSPI_Draw_Line(xs, ye, xe, ye, color_dat); // 画矩形下边
}

/*!
 * @brief    画圆
 * @param    x ：圆心x   (0-127)
 * @param    y ：圆心y   (0-159)
 * @param    r ：半径    (0-128)
 * @param    color_dat ：颜色
 * @return   无
 * @note     圆心坐标不要超出屏幕范围
 * @see      TFTSPI_Draw_Circle(50, 50, 30, u16YELLOW);
 * @date     2019/6/13 星期四
 */
void TFTSPI_Draw_Circle(u8 x, u8 y, u8 r, u16 color_dat)
{
    u8 dx, dy = r;

    if ((x >= r) && ((TFT18W - x) >= r) && (y >= r) && ((TFT18H - y) >= r)) // 确定所画圆在屏幕范围内，没有超出最外边，(暂不支持与屏幕边相交)
    {
        for (dx = 0; dx <= r; dx++)
        {
            while ((r * r + 1 - dx * dx) < (dy * dy))
                dy--;
            TFTSPI_Draw_Dot(x + dx, y - dy, color_dat);
            TFTSPI_Draw_Dot(x - dx, y - dy, color_dat);
            TFTSPI_Draw_Dot(x - dx, y + dy, color_dat);
            TFTSPI_Draw_Dot(x + dx, y + dy, color_dat);

            TFTSPI_Draw_Dot(x + dy, y - dx, color_dat);
            TFTSPI_Draw_Dot(x - dy, y - dx, color_dat);
            TFTSPI_Draw_Dot(x - dy, y + dx, color_dat);
            TFTSPI_Draw_Dot(x + dy, y + dx, color_dat);
        }
    }
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
 * @see      TFTSPI_Draw_Line(10, 20, 30, 40, u16YELLOW);
 * @date     2019/6/13 星期四
 */
void TFTSPI_Draw_Line(u8 xs, u8 ys, u8 xe, u8 ye, u16 color_dat)
{
    int i, ds;
    int dx, dy, inc_x, inc_y;
    int xerr = 0, yerr = 0; // 初始化变量

    if (xs == xe) // 如果是画垂直线则只需对竖直坐标计数
    {
        TFTSPI_Set_Pos(xs, ys, xe, ye);
        for (i = 0; i < (ye - ys + 1); i++)
            TFTSPI_Write_Word(color_dat);
    }
    else if (ys == ye) // 如果是水平线则只需要对水平坐标计数
    {
        TFTSPI_Set_Pos(xs, ys, xe, ye);
        for (i = 0; i < (xe - xs + 1); i++)
            TFTSPI_Write_Word(color_dat);
    }
    else // 如果是斜线，则重新计算，使用画点函数画出直线
    {
        dx = xe - xs; // 计算坐标增量
        dy = ye - ys;

        if (dx > 0)
            inc_x = 1; // 设置单步方向
        else
        {
            inc_x = -1;
            dx = -dx;
        }
        if (dy > 0)
            inc_y = 1; // 设置单步方向
        else
        {
            inc_y = -1;
            dy = -dy;
        }

        if (dx > dy)
        {
            ds = dx;
        } // 选取基本增量坐标轴
        else
        {
            ds = dy;
        }

        for (i = 0; i <= ds + 1; i++) // 画线输出
        {
            TFTSPI_Draw_Dot(xs, ys, color_dat); // 画点
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
}

/*!
 * @brief    画点
 * @param    x ：x
 * @param    y ：y
 * @param    color_dat ：颜色
 * @return   无
 * @note     起始、终止横坐标(0-127)，纵坐标(0-159),显示颜色uint16
 * @see      TFTSPI_Draw_Dot(10, 20, u16YELLOW);
 * @date     2019/6/13 星期四
 */
void TFTSPI_Draw_Dot(u8 x, u8 y, u16 color_dat)
{
    TFTSPI_Set_Pos(x, y, x, y);
    TFTSPI_Write_Word(color_dat);
}

/*!
 * @brief    显示图片
 * @param    xs ：起始x
 * @param    ys ：起始y
 * @param    xe ：结束x
 * @param    ys ：结束y
 * @param    ppic ：图片数据
 * @return   无
 * @note     起始、终止横坐标(0-127)，纵坐标(0-159),显示颜色uint16
 * @see
 * @date     2019/6/13 星期四
 */
void TFTSPI_Show_Pic(u8 xs, u8 ys, u8 xe, u8 ye, const u8 *ppic)
{
    u32 i, j, k = 0;

    TFTSPI_Set_Pos(xs, ys, xe - 1, ye);
    for (i = 0; i < ye - ys; i++)
    {
        for (j = 0; j < xe - xs; j++)
        {
            TFTSPI_Write_Word(((u16)ppic[k]) << 8 | ppic[k + 1]);
            k++;
            k++;
        }
    }
}

/*!
 * @brief    液晶字符输出(6*8字体)
 * @param    x: 0 - 20	(行)
 * @param    y: 0 - 19	(列)
 * @param    word_color: 字体颜色
 * @param    back_color: 背景颜色
 * @return   无
 * @note     内部调用
 * @see
 * @date     2019/6/13 星期四
 */
void TFTSPI_P6X8(u8 x, u8 y, u8 c_dat, u16 word_color, u16 back_color)
{
    u8 i, j;

    TFTSPI_Set_Pos(x * 6, y * 8, (x + 1) * 6 - 1, (y + 1) * 8 - 1); // 定位字符显示区域

    for (j = 0; j < 8; j++)
    {
        for (i = 0; i < 6; i++)
        {
            if ((Font_code8[c_dat - 32][i]) & (0x01 << j))
                TFTSPI_Write_Word(word_color);
            else
                TFTSPI_Write_Word(back_color);
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
 * @see      TFTSPI_P8X8(1, 1, "123456", u16YELLOW, u16RED);
 * @date     2019/6/13 星期四
 */
void TFTSPI_P8X8(u8 x, u8 y, u8 c_dat, u16 word_color, u16 back_color)
{
    u8 i, j;

    TFTSPI_Set_Pos(x * 8, y * 8, (x + 1) * 8 - 1, (y + 1) * 8 - 1); // 定位字符显示区域

    for (j = 0; j < 8; j++)
    {
        TFTSPI_Write_Word(back_color);
        for (i = 0; i < 6; i++)
        {
            if ((Font_code8[c_dat - 32][i]) & (0x01 << j))
                TFTSPI_Write_Word(word_color);
            else
                TFTSPI_Write_Word(back_color);
        }
        TFTSPI_Write_Word(back_color);
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
 * @see
 * @date     2019/6/13 星期四
 */
void TFTSPI_P8X16(u8 x, u8 y, u8 c_dat, u16 word_color, u16 back_color)
{
    u8 i, j;

    TFTSPI_Set_Pos(x * 8, y * 16, (x + 1) * 8 - 1, (y + 1) * 16 - 1); // 定位字符显示区域

    for (j = 0; j < 16; j++)
    {
        for (i = 0; i < 8; i++)
        {
            if ((Font_code16[c_dat - 32][j]) & (0x01 << i))
                TFTSPI_Write_Word(word_color);
            else
                TFTSPI_Write_Word(back_color);
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
 * @see      TFTSPI_P6X8Str(1, 1, "123456", u16YELLOW, u16RED);
 * @date     2019/6/13 星期四
 */
void TFTSPI_P6X8Str(u8 x, u8 y, char *s_dat, u16 word_color, u16 back_color)
{
    while (*s_dat)
        TFTSPI_P6X8(x++, y, *s_dat++, word_color, back_color);
}

/*!
 * @brief    液晶字符串输出(8*8字体)
 * @param    x:0 - 15 (行)
 * @param    y:0 - 19 (列)
 * @param    word_color: 字体颜色
 * @param    back_color: 背景颜色
 * @return   无
 * @note     无
 * @see      TFTSPI_P8X8Str(1, 1, "123456", u16YELLOW, u16RED);
 * @date     2019/6/13 星期四
 */
void TFTSPI_P8X8Str(u8 x, u8 y, char *s_dat, u16 word_color, u16 back_color)
{
    while (*s_dat)
        TFTSPI_P8X8(x++, y, *s_dat++, word_color, back_color);
}

/*!
 * @brief    液晶字符串输出(8*16字体)
 * @param    x: x: 0 -15   (行)
 * @param    y: y: 0 -9  	 (列)
 * @param    word_color: 字体颜色
 * @param    back_color: 背景颜色
 * @return   无
 * @note     无
 * @see      TFTSPI_P8X16Str(1, 1, "123456", u16YELLOW, u16RED);
 * @date     2019/6/13 星期四
 */
void TFTSPI_P8X16Str(u8 x, u8 y, char *s_dat, u16 word_color, u16 back_color)
{
    while (*s_dat)
        TFTSPI_P8X16(x++, y, *s_dat++, word_color, back_color);
}

/*!
 * @brief    液晶汉字字符串输出(16*16字体)
 * @param    x: 0 - 7	(行)
 * @param    y: 0 - 9	(列)
 * @param    word_color: 字体颜色
 * @param    back_color: 背景颜色
 * @return   无
 * @note     汉字只能是字库里的 字库没有的需要自行添加
 * @see      TFTSPI_P16x16Str(1, 1, "123456", u16YELLOW, u16RED);
 * @date     2019/6/13 星期四
 */
void TFTSPI_P16x16Str(u8 x, u8 y, u8 *s_dat, u16 word_color, u16 back_color)
{
    u8 wm = 0, ii = 0, i, j;
    int adder = 1;

    while (s_dat[ii] != '\0')
    {
        wm = 0;
        adder = 1;
        while (hanzi_Idx[wm] > 127)
        {
            if (hanzi_Idx[wm] == (u8)s_dat[ii])
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
            TFTSPI_Set_Pos(x * 16, y * 16, (x + 1) * 16 - 1, (y + 1) * 16 - 1); // 定位字符显示区域
            for (j = 0; j < 32; j++)
            {
                for (i = 0; i < 8; i++)
                {
                    if ((hanzi16x16[adder]) & (0x80 >> i))
                    {
                        TFTSPI_Write_Word(word_color);
                    }
                    else
                    {
                        TFTSPI_Write_Word(back_color);
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
}

/*!
 * @brief    显示图片
 * @param    xs ：起始x
 * @param    ys ：起始y
 * @param    xe ：结束x
 * @param    ys ：结束y
 * @param    ppic ：图片数据
 * @return   无
 * @note     起始、终止横坐标(0-127)，纵坐标(0-159),显示颜色uint16
 * @see
 * @date     2019/6/13 星期四
 */
void TFTSPI_Show_Pic2(u8 xs, u8 ys, u8 w, u8 h, const u8 *ppic)
{
    u32 i;
    TFTSPI_Set_Pos(xs, ys, xs + w - 1, ys + h);
    for (i = 0; i < w * h; i++)
        TFTSPI_Write_Word((ppic[2 * i] << 8) + (ppic[2 * i + 1])); // 高位在前，且两个数据组合成一个16位数据表示像素值
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
 * @date     2019/12/3 星期二
 */
void TFTSPI_Road(u8 wide_start, u8 high_start, u8 high, u8 wide, u8 *Pixle)
{
    u64 i;
    /* 设置显示范围 */
    TFTSPI_Set_Pos(wide_start, high_start, wide_start + wide - 1, high_start + high - 1);
    u16 color;
    u64 temp = high * wide;
    /* 显示图像 */
    for (i = 0; i < temp; i++)
    {
        /* 将灰度转化为 RGB565 */
        color = (Pixle[i] >> 3) << 11;
        color |= (Pixle[i] >> 2) << 5;
        color |= Pixle[i] >> 3;
        /* 显示 */
        TFTSPI_Write_Word(color);
    }
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
 * @date     2019/12/3 星期二
 */
void TFTSPI_BinRoad(u8 wide_start, u8 high_start, u8 high, u8 wide, u8 *Pixle)
{
    /* 设置显示范围 */
    TFTSPI_Set_Pos(wide_start, high_start, wide_start - 1 + wide, high_start - 1 + high);

    u64 temp = high * wide;
    /* 显示图像 */
    for (u64 i = 0; i < temp; i++)
    {
        if (Pixle[i])
            TFTSPI_Write_Word(0xffff); /* 显示 */
        else
            TFTSPI_Write_Word(0); /* 显示 */
    }
}
