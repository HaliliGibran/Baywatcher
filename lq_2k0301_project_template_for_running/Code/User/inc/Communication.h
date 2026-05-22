#ifndef __BAYWATCHER_COMMUNICATION_H_
#define __BAYWATCHER_COMMUNICATION_H_

#include "LQ_Uart.hpp"
#include <stddef.h>
#include <stdint.h>
#include <string>
#include <vector>

// 历史保留：菜单任务链仍用这组左右/直行编码表达预设绕行方向。
#define TURNING_LEFT     0
#define HEADING_FORWARD  1
#define TURNING_RIGHT    2

// 双板视觉状态码：
// - 识别板持续发送当前状态
// - 运行板只对 w/s/v 的状态上升沿触发原动作
enum class BoardVisionCode : uint8_t {
    INVALID = 0,
    VEHICLE = 'v',
    WEAPON  = 'w',
    SUPPLY  = 's',
    BRICK   = 'b',
    // u: 已命中识别标识型红块，但当前没有 v/w/s 结果输出
    NO_RESULT = 'u',
    // n: 检测区间内没有红色色块，或只有小的非标识红色色块
    UNKNOWN = 'n',
};

static constexpr uint8_t kBoardEventHeader1 = 0x5A;
static constexpr uint8_t kBoardEventHeader2 = 0xA5;
static constexpr uint8_t kBoardEventVersion = 0x01;
static constexpr uint8_t kBoardEventTail    = 0xED;

#pragma pack(push, 1)
struct BoardStatePacket {
    uint8_t header1;
    uint8_t header2;
    uint8_t version;
    uint8_t seq;
    uint8_t code;
    uint8_t crc8;
    uint8_t tail;
};
#pragma pack(pop)

class BoardComm {
public:
    BoardComm();
    ~BoardComm();

    // 双板通信统一走 UART1@115200。
    bool init(const std::string& port = UART1, uint32_t baud = B115200);
    bool send_state(BoardVisionCode code, uint8_t seq);
    bool try_receive_state(BoardVisionCode* out_code, uint8_t* out_seq);

private:
    LS_UART* uart_dev;
    std::vector<uint8_t> rx_cache_;

    uint8_t calculate_crc8(const uint8_t* data, size_t len) const;
    bool try_parse_cached_packet(BoardVisionCode* out_code, uint8_t* out_seq);
};

extern BoardComm comm;

#endif // !__BAYWATCHER_COMMUNICATION_H_

    /*  可选波特率：B0      B50     B75     B110    B134    B150
                   B200    B300    B600    B1200   B1800   B2400
                   B4800   B9600   B19200  B38400  B57600  B115200
                   B230400 B460800 B500000 B576000 B921600
    */
