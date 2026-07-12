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
    if (code == BoardVisionCode::INVALID) {
        return false;
    }
    return send_packet_code(static_cast<uint8_t>(code), seq);
}

bool BoardComm::send_recognition_gate(BoardRecognitionGate gate, uint8_t seq) {
    if (gate == BoardRecognitionGate::INVALID) {
        return false;
    }
    return send_packet_code(static_cast<uint8_t>(gate), seq);
}

bool BoardComm::send_packet_code(uint8_t code, uint8_t seq) {
    if (uart_dev == nullptr || code == 0u) {
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

bool BoardComm::try_parse_cached_packet(uint8_t* out_code, uint8_t* out_seq) {
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
        if (pkt.version == kBoardEventVersion &&
            pkt.tail == kBoardEventTail &&
            pkt.crc8 == expected_crc) {
            if (out_code != nullptr) {
                *out_code = pkt.code;
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

void BoardComm::read_into_cache() {
    if (uart_dev == nullptr) {
        return;
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
}

bool BoardComm::try_receive_state(BoardVisionCode* out_code, uint8_t* out_seq) {
    uint8_t raw_code = 0;
    uint8_t seq = 0;
    const auto take_cached_state = [&]() -> bool {
        while (try_parse_cached_packet(&raw_code, &seq)) {
            const bool valid_code =
                raw_code == static_cast<uint8_t>(BoardVisionCode::VEHICLE) ||
                raw_code == static_cast<uint8_t>(BoardVisionCode::WEAPON) ||
                raw_code == static_cast<uint8_t>(BoardVisionCode::SUPPLY) ||
                raw_code == static_cast<uint8_t>(BoardVisionCode::BRICK) ||
                raw_code == static_cast<uint8_t>(BoardVisionCode::BRICK_LEFT) ||
                raw_code == static_cast<uint8_t>(BoardVisionCode::BRICK_RIGHT) ||
                raw_code == static_cast<uint8_t>(BoardVisionCode::NO_RESULT) ||
                raw_code == static_cast<uint8_t>(BoardVisionCode::CLOTH_STOP) ||
                raw_code == static_cast<uint8_t>(BoardVisionCode::UNKNOWN);
            if (!valid_code) {
                continue;
            }
            if (out_code != nullptr) {
                *out_code = static_cast<BoardVisionCode>(raw_code);
            }
            if (out_seq != nullptr) {
                *out_seq = seq;
            }
            return true;
        }
        return false;
    };
    if (take_cached_state()) {
        return true;
    }
    read_into_cache();
    return take_cached_state();
}

bool BoardComm::try_receive_recognition_gate(BoardRecognitionGate* out_gate, uint8_t* out_seq) {
    uint8_t raw_code = 0;
    uint8_t seq = 0;
    const auto take_cached_gate = [&]() -> bool {
        while (try_parse_cached_packet(&raw_code, &seq)) {
            if (raw_code != static_cast<uint8_t>(BoardRecognitionGate::ALLOW) &&
                raw_code != static_cast<uint8_t>(BoardRecognitionGate::ALLOW_CIRCLE_RUNNING) &&
                raw_code != static_cast<uint8_t>(BoardRecognitionGate::BLOCK)) {
                continue;
            }
            if (out_gate != nullptr) {
                *out_gate = static_cast<BoardRecognitionGate>(raw_code);
            }
            if (out_seq != nullptr) {
                *out_seq = seq;
            }
            return true;
        }
        return false;
    };
    if (take_cached_gate()) {
        return true;
    }
    read_into_cache();
    return take_cached_gate();
}
