#include "Communication.h"
#include <stdio.h>

// ==================== 双板通信实现 ====================
BoardComm comm;

// 功能: 构造通信对象
// 类型: 成员函数（BoardComm）
// 关键参数: 无
BoardComm::BoardComm() : uart_dev(nullptr), rx_cache_() {}

// 功能: 析构通信对象
// 类型: 成员函数（BoardComm）
// 关键参数: 无
BoardComm::~BoardComm() {
    if (uart_dev != nullptr) {
        delete uart_dev;
        uart_dev = nullptr;
    }
}

// 功能: 初始化串口设备
// 类型: 成员函数（BoardComm）
// 关键参数: port-串口设备名, baud-波特率
bool BoardComm::init(const std::string& port, uint32_t baud) {
    if (uart_dev != nullptr) {
        delete uart_dev;
        uart_dev = nullptr;
    }

    uart_dev = new ls_uart(port, baud);
    rx_cache_.clear();

    if (uart_dev->flush_buffer()) {
        printf("[BoardComm] 双板通信串口 %s 就绪，波特率: %d\n", port.c_str(), baud);
        return true;
    } else {
        printf("[BoardComm] 串口初始化失败，请检查硬件引脚或权限！\n");
        return false;
    }
}

// 功能: 计算事件包 CRC8
// 类型: 成员函数（BoardComm）
// 关键参数: data/len-待校验字节流
uint8_t BoardComm::calculate_crc8(const uint8_t* data, size_t len) const {
    uint8_t crc = 0x00;
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; ++bit) {
            if ((crc & 0x80u) != 0u) {
                crc = static_cast<uint8_t>((crc << 1) ^ 0x07u);
            } else {
                crc = static_cast<uint8_t>(crc << 1);
            }
        }
    }
    return crc;
}

// 功能: 发送识别动作事件
// 类型: 成员函数（BoardComm）
// 关键参数: action/seq-一次性识别事件
bool BoardComm::send_event(BoardActionEvent action, uint8_t seq) {
    if (uart_dev == nullptr || action == BoardActionEvent::NONE) {
        return false;
    }

    BoardEventPacket pkt{};
    pkt.header1 = kBoardEventHeader1;
    pkt.header2 = kBoardEventHeader2;
    pkt.version = kBoardEventVersion;
    pkt.seq = seq;
    pkt.action = static_cast<uint8_t>(action);
    pkt.crc8 = calculate_crc8(&pkt.version, 3);
    pkt.tail = kBoardEventTail;

    return uart_dev->write_data(reinterpret_cast<const uint8_t*>(&pkt), sizeof(BoardEventPacket)) ==
           static_cast<ssize_t>(sizeof(BoardEventPacket));
}

// 功能: 从缓存字节流里解析一帧合法事件
// 类型: 成员函数（BoardComm）
// 关键参数: out_action/out_seq-输出解析成功后的事件
bool BoardComm::try_parse_cached_packet(BoardActionEvent* out_action, uint8_t* out_seq) {
    const size_t kPacketSize = sizeof(BoardEventPacket);

    while (rx_cache_.size() >= kPacketSize) {
        if (rx_cache_[0] != kBoardEventHeader1 || rx_cache_[1] != kBoardEventHeader2) {
            rx_cache_.erase(rx_cache_.begin());
            continue;
        }

        BoardEventPacket pkt{};
        for (size_t i = 0; i < kPacketSize; ++i) {
            reinterpret_cast<uint8_t*>(&pkt)[i] = rx_cache_[i];
        }

        const uint8_t expected_crc = calculate_crc8(&pkt.version, 3);
        const bool valid_action =
            pkt.action == static_cast<uint8_t>(BoardActionEvent::WEAPON) ||
            pkt.action == static_cast<uint8_t>(BoardActionEvent::SUPPLY) ||
            pkt.action == static_cast<uint8_t>(BoardActionEvent::VEHICLE);

        if (pkt.version == kBoardEventVersion &&
            pkt.tail == kBoardEventTail &&
            pkt.crc8 == expected_crc &&
            valid_action) {
            if (out_action != nullptr) {
                *out_action = static_cast<BoardActionEvent>(pkt.action);
            }
            if (out_seq != nullptr) {
                *out_seq = pkt.seq;
            }
            rx_cache_.erase(rx_cache_.begin(), rx_cache_.begin() + static_cast<ptrdiff_t>(kPacketSize));
            return true;
        }

        rx_cache_.erase(rx_cache_.begin());
    }

    return false;
}

// 功能: 读取串口字节流并尝试解析事件
// 类型: 成员函数（BoardComm）
// 关键参数: out_action/out_seq-输出事件
bool BoardComm::try_receive_event(BoardActionEvent* out_action, uint8_t* out_seq) {
    if (uart_dev == nullptr) {
        return false;
    }

    uint8_t rx_buffer[64] = {0};
    const ssize_t bytes_read = uart_dev->read_data(rx_buffer, sizeof(rx_buffer));
    if (bytes_read > 0) {
        rx_cache_.insert(rx_cache_.end(), rx_buffer, rx_buffer + bytes_read);
        if (rx_cache_.size() > 256u) {
            rx_cache_.erase(rx_cache_.begin(),
                            rx_cache_.end() - static_cast<ptrdiff_t>(sizeof(BoardEventPacket)));
        }
    }

    return try_parse_cached_packet(out_action, out_seq);
}
