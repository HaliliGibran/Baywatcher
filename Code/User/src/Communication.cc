#include "Communication.h"
#include <stdio.h>

BoardComm::BoardComm() : uart_dev(nullptr) {}

BoardComm::~BoardComm() {
    if (uart_dev != nullptr) {
        delete uart_dev;
        uart_dev = nullptr;
    }
}

bool BoardComm::init(const std::string& port, uint32_t baud) {
    uart_dev = new ls_uart(port, baud);
    
    if (uart_dev->flush_buffer()) {
        printf("[BoardComm] 双板通信串口 %s 就绪，波特率: %d\n", port.c_str(), baud);
        return true;
    } else {
        printf("[BoardComm] 串口初始化失败，请检查硬件引脚或权限！\n");
        return false;
    }
}

// 针对你定义的3个业务字节计算校验和
uint8_t BoardComm::calculate_checksum(const CommPacket& pkt) {
    uint8_t sum = 0;
    sum += pkt.work_mode;
    sum += pkt.board_check;
    sum += pkt.turning_method;
    return sum;
}

// 视觉板调用此函数发送命令
void BoardComm::send_data(uint8_t mode, uint8_t board_status, uint8_t turn_dir) {
    if (uart_dev == nullptr) return;

    CommPacket pkt;
    pkt.header1 = 0x5A;
    pkt.header2 = 0xA5;
    
    // 装填视觉识别到的业务数据
    pkt.work_mode      = mode;
    pkt.board_check    = board_status;
    pkt.turning_method = turn_dir;
    
    // 计算校验和并封尾
    pkt.checksum = calculate_checksum(pkt);
    pkt.tail = 0xED;

    // 一次性发送整个结构体
    uart_dev->write_data((const uint8_t*)&pkt, sizeof(CommPacket)); 
}

// 底盘控制板调用此函数接收命令
bool BoardComm::receive_data(CommPacket& out_packet) {
    if (uart_dev == nullptr) return false;

    uint8_t rx_buffer[128]; 
    ssize_t bytes_read = uart_dev->read_data(rx_buffer, sizeof(CommPacket)); 

    // 如果读取到了完整长度的一帧数据
    if (bytes_read == sizeof(CommPacket)) {
        CommPacket* temp_pkt = (CommPacket*)rx_buffer;

        // 1. 验证帧头和帧尾
        if (temp_pkt->header1 == 0x5A && temp_pkt->header2 == 0xA5 && temp_pkt->tail == 0xED) {
            // 2. 验证校验和，防止串口线干扰导致的位翻转
            if (temp_pkt->checksum == calculate_checksum(*temp_pkt)) {
                out_packet = *temp_pkt; 
                return true;
            }
        }
        // 如果数据错位或损坏，清空缓冲区重新对齐
        uart_dev->flush_buffer(); 
    }
    return false;
}