#ifndef __BAYWATCHER_COMMUNICATION_H_
#define __BAYWATCHER_COMMUNICATION_H_

#include "lq_uart.hpp"
#include <stddef.h>
#include <stdint.h>
#include <string>
#include <vector>

#define TURNING_LEFT     0
#define HEADING_FORWARD  1
#define TURNING_RIGHT    2

// 双板视觉状态码：
// - 识别板持续回传当前观察到的状态
// - 运行板对 w/s/v 触发绕行动作，对 u/c 做速度覆盖，对 bl/br 做砖块避让
enum class BoardVisionCode : uint8_t {
    INVALID = 0,
    VEHICLE = 'v',
    WEAPON  = 'w',
    SUPPLY  = 's',
    // 兼容旧砖块状态：无侧别，只做旧砖块处理。
    BRICK   = 'b',
    // bl/br: 红砖在左/右侧。串口仍是单字节码，日志显示为 bl/br。
    BRICK_LEFT = 'L',
    BRICK_RIGHT = 'R',
    // u: 已命中识别标识型红块，但当前没有 v/w/s 结果输出
    NO_RESULT = 'u',
    // c: 软件盲盒色布发车停车状态
    CLOTH_STOP = 'c',
    // n: 检测区间内没有红色色块，或只有小的非标识红色色块
    UNKNOWN = 'n',
};

// 运行板反向发送给识别板的识别门控。
enum class BoardRecognitionGate : uint8_t {
    INVALID = 0,
    ALLOW = 'A',
    ALLOW_CIRCLE_RUNNING = 'C',
    BLOCK = 'X',
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

    bool init(const std::string& port = UART1, uint32_t baud = B115200);
    bool send_state(BoardVisionCode code, uint8_t seq);
    bool try_receive_state(BoardVisionCode* out_code, uint8_t* out_seq);
    bool send_recognition_gate(BoardRecognitionGate gate, uint8_t seq);
    bool try_receive_recognition_gate(BoardRecognitionGate* out_gate, uint8_t* out_seq);

private:
    ls_uart* uart_dev;
    std::vector<uint8_t> rx_cache_;

    uint8_t calculate_crc8(const uint8_t* data, size_t len) const;
    bool send_packet_code(uint8_t code, uint8_t seq);
    void read_into_cache();
    bool try_parse_cached_packet(uint8_t* out_code, uint8_t* out_seq);
};

extern BoardComm comm;

#endif // !__BAYWATCHER_COMMUNICATION_H_

    /*  可选波特率：B0      B50     B75     B110    B134    B150
                   B200    B300    B600    B1200   B1800   B2400
                   B4800   B9600   B19200  B38400  B57600  B115200
                   B230400 B460800 B500000 B576000 B921600
    */
