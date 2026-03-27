#ifndef __BAYWATCHER_COMMUNICATION_H_
#define __BAYWATCHER_COMMUNICATION_H_

#include "lq_uart.hpp"
#include <stddef.h>
#include <stdint.h>
#include <string>
#include <vector>

// 历史保留：菜单任务链仍用这组左右/直行编码表达预设绕行方向。
#define TURNING_LEFT     0
#define HEADING_FORWARD  1
#define TURNING_RIGHT    2

// 双板识别事件枚举：
// - NONE 只在本地状态机里表示“当前没有事件”，不上串口。
// - 识别板只发一次性事件；运行板收到后按事件 owner 执行动作。
enum class BoardActionEvent : uint8_t {
    NONE    = 0,
    WEAPON  = 1,
    SUPPLY  = 2,
    VEHICLE = 3,
};

// 双板事件协议固定常量。
static constexpr uint8_t kBoardEventHeader1 = 0x5A;
static constexpr uint8_t kBoardEventHeader2 = 0xA5;
static constexpr uint8_t kBoardEventVersion = 0x01;
static constexpr uint8_t kBoardEventTail    = 0xED;

// 1字节对齐，防止编译器自动填充导致数据包大小不一致。
#pragma pack(push, 1)
struct BoardEventPacket {
    uint8_t header1;  // 帧头1
    uint8_t header2;  // 帧头2
    uint8_t version;  // 协议版本
    uint8_t seq;      // 事件序号，识别板每产生一个新事件就递增一次
    uint8_t action;   // BoardActionEvent
    uint8_t crc8;     // 对 version/seq/action 做 CRC8
    uint8_t tail;     // 帧尾
};
#pragma pack(pop)

class BoardComm {
public:
    BoardComm();
    ~BoardComm();

    // 双板通信统一走 UART1@115200。
    bool init(const std::string& port = UART1, uint32_t baud = B115200);

    // 识别板发送一次动作事件。
    bool send_event(BoardActionEvent action, uint8_t seq);

    // 运行板从串口字节流中解析出一帧完整事件。
    bool try_receive_event(BoardActionEvent* out_action, uint8_t* out_seq);

private:
    ls_uart* uart_dev;
    std::vector<uint8_t> rx_cache_;

    uint8_t calculate_crc8(const uint8_t* data, size_t len) const;
    bool try_parse_cached_packet(BoardActionEvent* out_action, uint8_t* out_seq);
};

extern BoardComm comm;

#endif // !__BAYWATCHER_COMMUNICATION_H_

    /*  可选波特率：B0      B50     B75     B110    B134    B150
                   B200    B300    B600    B1200   B1800   B2400
                   B4800   B9600   B19200  B38400  B57600  B115200
                   B230400 B460800 B500000 B576000 B921600
    */
