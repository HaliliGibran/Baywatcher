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

enum class BoardActionEvent : uint8_t {
    NONE    = 0,
    WEAPON  = 1,
    SUPPLY  = 2,
    VEHICLE = 3,
};

static constexpr uint8_t kBoardEventHeader1 = 0x5A;
static constexpr uint8_t kBoardEventHeader2 = 0xA5;
static constexpr uint8_t kBoardEventVersion = 0x01;
static constexpr uint8_t kBoardEventTail    = 0xED;

#pragma pack(push, 1)
struct BoardEventPacket {
    uint8_t header1;
    uint8_t header2;
    uint8_t version;
    uint8_t seq;
    uint8_t action;
    uint8_t crc8;
    uint8_t tail;
};
#pragma pack(pop)

class BoardComm {
public:
    BoardComm();
    ~BoardComm();

    bool init(const std::string& port = UART1, uint32_t baud = B115200);
    bool send_event(BoardActionEvent action, uint8_t seq);
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
