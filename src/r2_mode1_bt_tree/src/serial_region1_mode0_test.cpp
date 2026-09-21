#include <serial/serial.h>

#include <array>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace {

constexpr uint8_t kFrameHeader = 0xAA;
constexpr uint8_t kFrameTail = 0x55;
constexpr std::size_t kPayloadLength = 7;
constexpr std::size_t kFrameLength = 11;

struct Options {
    std::string port = "/dev/ttyUSB1";
    uint32_t baudrate = 115200;
    uint32_t timeout_ms = 3000;
    bool send_request = true;
};

std::array<uint8_t, 2> modbusCrc16(const std::vector<uint8_t>& data) {
    uint16_t crc = 0xFFFF;
    for (uint8_t byte : data) {
        crc ^= byte;
        for (int i = 0; i < 8; ++i) {
            if (crc & 0x0001U) {
                crc = static_cast<uint16_t>((crc >> 1U) ^ 0xA001U);
            } else {
                crc >>= 1U;
            }
        }
    }
    return {static_cast<uint8_t>(crc & 0xFFU), static_cast<uint8_t>((crc >> 8U) & 0xFFU)};
}

std::vector<uint8_t> buildFrame(uint8_t region, uint8_t mode, uint8_t need_return, int16_t data_value) {
    std::vector<uint8_t> payload;
    payload.reserve(kPayloadLength);
    payload.push_back(region);
    payload.push_back(mode);
    payload.push_back(need_return);
    payload.push_back(static_cast<uint8_t>(data_value & 0xFF));
    payload.push_back(static_cast<uint8_t>((data_value >> 8) & 0xFF));
    payload.push_back(0x00);
    payload.push_back(0x00);

    const auto crc = modbusCrc16(payload);

    std::vector<uint8_t> frame;
    frame.reserve(kFrameLength);
    frame.push_back(kFrameHeader);
    frame.insert(frame.end(), payload.begin(), payload.end());
    frame.push_back(crc[0]);
    frame.push_back(crc[1]);
    frame.push_back(kFrameTail);
    return frame;
}

std::string toHex(const std::vector<uint8_t>& data) {
    std::ostringstream out;
    out << std::uppercase << std::hex << std::setfill('0');
    for (std::size_t i = 0; i < data.size(); ++i) {
        if (i != 0U) {
            out << ' ';
        }
        out << std::setw(2) << static_cast<int>(data[i]);
    }
    return out.str();
}

bool parseUint32(const std::string& value, uint32_t& output) {
    try {
        const auto parsed = std::stoul(value);
        if (parsed > UINT32_MAX) {
            return false;
        }
        output = static_cast<uint32_t>(parsed);
        return true;
    } catch (const std::exception&) {
        return false;
    }
}

void printUsage(const char* program) {
    std::cerr
        << "Usage: " << program << " [--port /dev/ttyUSB1] [--baud 115200] [--timeout-ms 3000] [--listen-only]\n"
        << "\n"
        << "Default behavior sends one request frame: region=1, mode=0, need_return=1, data=0,\n"
        << "then waits for a valid reply frame whose payload is region=1 and mode=0.\n";
}

Options parseArgs(int argc, char** argv) {
    Options options;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--help" || arg == "-h") {
            printUsage(argv[0]);
            std::exit(0);
        }
        if (arg == "--port" && i + 1 < argc) {
            options.port = argv[++i];
            continue;
        }
        if (arg == "--baud" && i + 1 < argc) {
            if (!parseUint32(argv[++i], options.baudrate)) {
                throw std::runtime_error("invalid --baud value");
            }
            continue;
        }
        if (arg == "--timeout-ms" && i + 1 < argc) {
            if (!parseUint32(argv[++i], options.timeout_ms)) {
                throw std::runtime_error("invalid --timeout-ms value");
            }
            continue;
        }
        if (arg == "--listen-only") {
            options.send_request = false;
            continue;
        }
        throw std::runtime_error("unknown argument: " + arg);
    }
    return options;
}

bool popFrame(std::vector<uint8_t>& rx_buffer, std::vector<uint8_t>& frame) {
    while (!rx_buffer.empty() && rx_buffer.front() != kFrameHeader) {
        rx_buffer.erase(rx_buffer.begin());
    }

    if (rx_buffer.size() < kFrameLength) {
        return false;
    }

    if (rx_buffer[kFrameLength - 1] != kFrameTail) {
        rx_buffer.erase(rx_buffer.begin());
        return false;
    }

    frame.assign(rx_buffer.begin(), rx_buffer.begin() + kFrameLength);
    rx_buffer.erase(rx_buffer.begin(), rx_buffer.begin() + kFrameLength);
    return true;
}

bool isValidRegion1Mode0Reply(const std::vector<uint8_t>& frame) {
    if (frame.size() != kFrameLength || frame.front() != kFrameHeader || frame.back() != kFrameTail) {
        return false;
    }

    const std::vector<uint8_t> payload(frame.begin() + 1, frame.begin() + 1 + static_cast<long>(kPayloadLength));
    const auto crc = modbusCrc16(payload);
    if (frame[8] != crc[0] || frame[9] != crc[1]) {
        std::cout << "收到帧但 CRC 错误: " << toHex(frame) << '\n';
        return false;
    }

    const uint8_t region = payload[0];
    const uint8_t mode = payload[1];
    const uint8_t task_complete = payload[2];
    const int16_t data_value = static_cast<int16_t>(payload[3] | (payload[4] << 8));

    std::cout << "收到有效帧: region=" << static_cast<int>(region)
              << ", mode=" << static_cast<int>(mode)
              << ", task_complete=" << static_cast<int>(task_complete)
              << ", data=" << data_value
              << ", raw=" << toHex(frame) << '\n';

    return region == 0x01U && mode == 0x00U;
}

}  // namespace

int main(int argc, char** argv) {
    Options options;
    try {
        options = parseArgs(argc, argv);
    } catch (const std::exception& e) {
        std::cerr << e.what() << '\n';
        printUsage(argv[0]);
        return 2;
    }

    serial::Serial serial_port;
    try {
        serial_port.setPort(options.port);
        serial_port.setBaudrate(options.baudrate);
        auto timeout = serial::Timeout::simpleTimeout(20);
        serial_port.setTimeout(timeout);
        serial_port.open();
    } catch (const std::exception& e) {
        std::cerr << "串口打开失败: " << e.what() << '\n';
        return 2;
    }

    std::cout << "串口已打开: " << options.port << " @ " << options.baudrate << '\n';

    if (options.send_request) {
        const auto request = buildFrame(0x01, 0x00, 0x01, 0);
        try {
            serial_port.flushOutput();
            serial_port.write(request);
            serial_port.flush();
            std::cout << "已发送区域1模式0测试请求: " << toHex(request) << '\n';
        } catch (const std::exception& e) {
            std::cerr << "串口发送失败: " << e.what() << '\n';
            return 2;
        }
    } else {
        std::cout << "listen-only 模式: 不发送请求，只等待回包\n";
    }

    std::vector<uint8_t> rx_buffer;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(options.timeout_ms);

    while (std::chrono::steady_clock::now() < deadline) {
        try {
            const auto available = serial_port.available();
            if (available > 0U) {
                const auto bytes = serial_port.read(available);
                if (!bytes.empty()) {
                    const std::vector<uint8_t> read_bytes(bytes.begin(), bytes.end());
                    rx_buffer.insert(rx_buffer.end(), read_bytes.begin(), read_bytes.end());
                    std::cout << "收到原始数据: " << toHex(read_bytes) << '\n';
                }
            }

            std::vector<uint8_t> frame;
            while (popFrame(rx_buffer, frame)) {
                if (isValidRegion1Mode0Reply(frame)) {
                    std::cout << "测试通过: 收到区域1模式0回包\n";
                    return 0;
                }
            }
        } catch (const std::exception& e) {
            std::cerr << "串口读取失败: " << e.what() << '\n';
            return 2;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    std::cout << "测试失败: " << options.timeout_ms << " ms 内未收到区域1模式0回包\n";
    return 1;
}
