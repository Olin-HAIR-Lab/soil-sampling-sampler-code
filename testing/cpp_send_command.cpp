#include <iostream>
#include <vector>
#include <thread>
#include <chrono>
#include <iomanip>
#include <cstring>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sstream>
#include <cstdint>

int serial_port;

const int32_t START_SIGNAL = 123456789;
const int32_t END_SIGNAL   = 987654321;

uint8_t calculate_checksum(const std::vector<uint8_t>& data) {
    int sum = 0;
    for (size_t i = 2; i < data.size(); ++i) {
        sum += data[i];
    }
    return static_cast<uint8_t>(sum & 0xFF);
}

int32_t read_i32_le(const uint8_t* p) {
    return (int32_t)(
        ((uint32_t)p[0]) |
        ((uint32_t)p[1] << 8) |
        ((uint32_t)p[2] << 16) |
        ((uint32_t)p[3] << 24)
    );
}

bool can_setup() {
    std::vector<uint8_t> packet = {
        0xAA, // Packet header
        0x55, // Packet header
        0x12, // Variable protocol setting
        0x03, // CAN baud rate: 500 kbps
        0x01, // Standard frame
        0x00, // Filter ID1
        0x00, // Filter ID2
        0x00, // Filter ID3
        0x00, // Filter ID4
        0x00, // Mask ID1
        0x00, // Mask ID2
        0x00, // Mask ID3
        0x00, // Mask ID4
        0x00, // CAN mode: normal
        0x00, // Automatic resend
        0x00, // Spare bytes at the end
        0x00, 
        0x00, 
        0x00  
    };

    packet.push_back(calculate_checksum(packet));

    ssize_t n = write(serial_port, packet.data(), packet.size());
    if (n < 0) {
        perror("Error writing CAN setup packet");
        return false;
    }

    tcdrain(serial_port);

    std::cout << "Sent CAN setup packet (" << n << " bytes): ";
    for (uint8_t b : packet) {
        printf("%02X ", b);
    }
    std::cout << std::endl;

    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    return true;
}

void can_send(int signal) {
    uint8_t signal_b[4];
    std::memcpy(signal_b, &signal, sizeof(signal));

    std::vector<uint8_t> packet = {
        0xAA,
        0xC8,       // standard data frame, DLC=8
        0x12,       // standard ID low byte
        0x34,       // standard ID high byte
        signal_b[0],
        signal_b[1],
        signal_b[2],
        signal_b[3],
        0x00,
        0x00,
        0x00,
        0x00,
        0x55
    };

    ssize_t n = write(serial_port, packet.data(), packet.size());
    if (n < 0) {
        perror("Error writing CAN send packet");
        return;
    }

    tcdrain(serial_port); // drain serial port to ensure write goes through

    std::cout << "Sent Signal: " << signal << " as bytes: ";
    for (uint8_t b : packet) {
        printf("%02X ", b);
    }
    std::cout << std::endl;
}

bool try_parse_frame(std::vector<uint8_t>& buffer, int32_t& a, int32_t& b) {
    // Need at least minimal frame (13 bytes)
    if (buffer.size() < 13) return false;

    // Look for header
    if (buffer[0] != 0xAA) {
        buffer.erase(buffer.begin());
        return false;
    }

    uint8_t type = buffer[1];
    if ((type & 0xC0) != 0xC0) { // check that the frame is valid from the type byte (top two bits)
        buffer.erase(buffer.begin());
        return false;
    }

    int dlc = type & 0x0F; // bottom 4 bits hold the length of data enclosed

    int expected_len = 2 + 2 + dlc + 1; // header+type + id(2) + data + end
    if ((int)buffer.size() < expected_len) return false;

    if (buffer[expected_len - 1] != 0x55) { // frame needs to end with the terminator
        buffer.erase(buffer.begin());
        return false;
    }

    // Extract data
    int data_start = 4;
    if (dlc < 8) return false;

    a = read_i32_le(&buffer[data_start]);
    b = read_i32_le(&buffer[data_start + 4]);

    buffer.erase(buffer.begin(), buffer.begin() + expected_len);
    return true;
}

int main() {
    serial_port = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY);
    if (serial_port < 0) {
        perror("open");
        return 1;
    }

    struct termios tty{};
    tcgetattr(serial_port, &tty);
    cfmakeraw(&tty);
    cfsetospeed(&tty, B2000000);
    cfsetispeed(&tty, B2000000);

    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;

    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 1;

    tcsetattr(serial_port, TCSANOW, &tty);
    tcflush(serial_port, TCIOFLUSH);

    can_setup();
    can_send(7); // Corresponds to the full sampling routine command

    std::vector<uint8_t> buffer;
    std::vector<int32_t> received_data;

    bool receiving = false;

    char temp[256];

    std::cout << "Waiting for data..." << std::endl;

    while (true) {
        int n = read(serial_port, temp, sizeof(temp));
        if (n > 0) {
            buffer.insert(buffer.end(), temp, temp + n);
        }

        int32_t a, b;
        // process returned data when it comes back
        while (try_parse_frame(buffer, a, b)) {
            if (a == START_SIGNAL) {
                receiving = true;
                received_data.clear();
                std::cout << "START detected\n";
            }
            else if (a == END_SIGNAL) {
                receiving = false;

                std::cout << "END detected\n";
                std::cout << "Received data:\n[ ";
                for (auto v : received_data) {
                    std::cout << v << " "; // print elements of array
                }
                std::cout << "]\n\n";
            }
            else if (receiving) {
                received_data.push_back(a);
                if (b != 0) {
                    received_data.push_back(b);
                }
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    close(serial_port);
    return 0;
}