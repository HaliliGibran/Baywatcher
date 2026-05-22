#include "Communication.h"
#include <stdio.h>

// ==================== 双板通信实现 ====================
BoardComm comm;

namespace {

static uint32_t board_comm_baud_to_hz(uint32_t baud)
{
    switch (baud) {
    case B9600: return 9600u;
    case B19200: return 19200u;
    case B38400: return 38400u;
    case B57600: return 57600u;
    case B115200: return 115200u;
#ifdef B230400
    case B230400: return 230400u;
#endif
#ifdef B460800
    case B460800: return 460800u;
#endif
#ifdef B921600
    case B921600: return 921600u;
#endif
    default: return baud;
    }
}

} // namespace

BoardComm::BoardComm() : uart_dev(nullptr), rx_cache_() {}

BoardComm::~BoardComm() {
    if (uart_dev != nullptr) {
        delete uart_dev;
        uart_dev = nullptr;
    }
}

bool BoardComm::init(const std::string& port, uint32_t baud) {
    if (uart_dev != nullptr) {
        delete uart_dev;
        uart_dev = nullptr;
    }

    uart_dev = new LS_UART(port, baud, LS_UART_STOP1, LS_UART_DATA8, LS_UART_NONE);
    rx_cache_.clear();
    printf("[BoardComm] uart ready: %s @ %u\n",
           port.c_str(),
           board_comm_baud_to_hz(baud));
    return true;
}

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

bool BoardComm::send_state(BoardVisionCode code, uint8_t seq) {
    if (uart_dev == nullptr || code == BoardVisionCode::INVALID) {
        return false;
    }

    BoardStatePacket pkt{};
    pkt.header1 = kBoardEventHeader1;
    pkt.header2 = kBoardEventHeader2;
    pkt.version = kBoardEventVersion;
    pkt.seq = seq;
    pkt.code = static_cast<uint8_t>(code);
    pkt.crc8 = calculate_crc8(&pkt.version, 3);
    pkt.tail = kBoardEventTail;

    return uart_dev->WriteData(reinterpret_cast<const char*>(&pkt), sizeof(BoardStatePacket)) ==
           static_cast<ssize_t>(sizeof(BoardStatePacket));
}

bool BoardComm::try_parse_cached_packet(BoardVisionCode* out_code, uint8_t* out_seq) {
    const size_t kPacketSize = sizeof(BoardStatePacket);

    while (rx_cache_.size() >= kPacketSize) {
        if (rx_cache_[0] != kBoardEventHeader1 || rx_cache_[1] != kBoardEventHeader2) {
            rx_cache_.erase(rx_cache_.begin());
            continue;
        }

        BoardStatePacket pkt{};
        for (size_t i = 0; i < kPacketSize; ++i) {
            reinterpret_cast<uint8_t*>(&pkt)[i] = rx_cache_[i];
        }

        const uint8_t expected_crc = calculate_crc8(&pkt.version, 3);
        const bool valid_code =
            pkt.code == static_cast<uint8_t>(BoardVisionCode::VEHICLE) ||
            pkt.code == static_cast<uint8_t>(BoardVisionCode::WEAPON) ||
            pkt.code == static_cast<uint8_t>(BoardVisionCode::SUPPLY) ||
            pkt.code == static_cast<uint8_t>(BoardVisionCode::BRICK) ||
            pkt.code == static_cast<uint8_t>(BoardVisionCode::NO_RESULT) ||
            pkt.code == static_cast<uint8_t>(BoardVisionCode::UNKNOWN);

        if (pkt.version == kBoardEventVersion &&
            pkt.tail == kBoardEventTail &&
            pkt.crc8 == expected_crc &&
            valid_code) {
            if (out_code != nullptr) {
                *out_code = static_cast<BoardVisionCode>(pkt.code);
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

bool BoardComm::try_receive_state(BoardVisionCode* out_code, uint8_t* out_seq) {
    if (uart_dev == nullptr) {
        return false;
    }

    uint8_t rx_buffer[64] = {0};
    const ssize_t bytes_read = uart_dev->ReadData(reinterpret_cast<char*>(rx_buffer), sizeof(rx_buffer));
    if (bytes_read > 0) {
        rx_cache_.insert(rx_cache_.end(), rx_buffer, rx_buffer + bytes_read);
        if (rx_cache_.size() > 256u) {
            rx_cache_.erase(rx_cache_.begin(),
                            rx_cache_.end() - static_cast<ptrdiff_t>(sizeof(BoardStatePacket)));
        }
    }

    return try_parse_cached_packet(out_code, out_seq);
}
