#ifndef __LQ_IMG_TRANS_HTTP_HPP
#define __LQ_IMG_TRANS_HTTP_HPP

#include <iostream>
#include <cstring>
#include <cstdio>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

// extern "C" {
// #include <libavformat/avformat.h>
// #include <libavcodec/avcodec.h>
// #include <libavdevice/avdevice.h>
// #include <libavutil/opt.h>
// }

/****************************************************************************************************
 * @brief   枚举定义
 ****************************************************************************************************/

// 帧延迟模式枚举
typedef enum frame_delay_opt
{
    FRAME_CONT_MODE = 0x00, // 连续模式
    FRAME_LOSS_MODE,        // 丢帧模式(只发送最新帧)
} frame_delay_opt_t;

/****************************************************************************************************
 * @brief   类定义
 ****************************************************************************************************/

class lq_img_trans_http
{
public:
    lq_img_trans_http();            // 无参构造函数
    ~lq_img_trans_http();           // 析构函数

public:

private:
    size_t            width_;           // 图像宽度    
    size_t            height_;          // 图像高度
    size_t            fps_;             // 帧率
    size_t            port_;            // 服务端口号
    frame_delay_opt_t frame_delay_opt_; // 帧延迟模式
};

#endif
