#ifndef __BAYWATCHER_COMMUNICATION_H_
#define __BAYWATCHER_COMMUNICATION_H_

#include "lq_uart.hpp" 
#include <stdint.h>

// 绕行方向宏定义
#define TURNING_LEFT     0
#define HEADING_FORWARD  1
#define TURNING_RIGHT    2

// 1字节对齐，防止编译器自动填充导致数据包大小不一致
#pragma pack(push, 1) 
struct CommPacket {
    uint8_t header1;    // 帧头1: 0x5A
    uint8_t header2;    // 帧头2: 0xA5
    
    // --- 业务数据区 ---
    uint8_t work_mode;       // 运行模式: 0正常巡线 / 1减速模式 / 2绕行方式
    uint8_t board_check;     // 红色目标板状态: 0未识别到 / 1已识别到
    uint8_t turning_method;  // 绕行方式: 0左行 / 1直行 / 2右行
    // ------------------------------------

    uint8_t checksum;   // 校验和 (防数据篡改)
    uint8_t tail;       // 帧尾: 0xED
};
#pragma pack(pop)

class BoardComm {
public:
    BoardComm();
    ~BoardComm();

    // 初始化串口，默认使用库里定义的 UART1 ("/dev/ttyS1")
    bool init(const std::string& port = UART1, uint32_t baud = B115200); 

    // 发送数据函数 
    void send_data(uint8_t mode, uint8_t board_status, uint8_t turn_dir);

    // 接收并解析数据函数，如果有有效新数据返回 true
    bool receive_data(CommPacket& out_packet);

private:
    ls_uart* uart_dev;
    
    // 计算校验和的辅助函数
    uint8_t calculate_checksum(const CommPacket& pkt);
};

extern BoardComm comm;

#endif // !__BAYWATCHER_COMMUNICATION_H_

    /*  可选波特率：B0      B50     B75     B110    B134    B150
                   B200    B300    B600    B1200   B1800   B2400
                   B4800   B9600   B19200  B38400  B57600  B115200
                   B230400 B460800 B500000 B576000 B921600
    */
