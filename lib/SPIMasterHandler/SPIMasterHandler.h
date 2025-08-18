#ifndef SPI_MASTER_HANDLER_H_
#define SPI_MASTER_HANDLER_H_

#include <cstdint>
#include <cstddef>
#include <cstring>
#include <chrono>
#include <stdexcept>

#define SPI_HEADER_MASTER 0x55
#define SPI_HEADER_SLAVE  0x45
#define SPI_NUM_FLOATS    3
#define SPI_MSG_SIZE      (1 + SPI_NUM_FLOATS * 4 + 1)

using namespace std::chrono;

// Mirror of your mbed struct
struct SPIData {
    float    data[SPI_NUM_FLOATS];
    uint32_t message_count;
    uint32_t failed_count;
    uint32_t last_delta_time_us;
    uint32_t readout_time_us;

    SPIData()
        : message_count(0),
          failed_count(0),
          last_delta_time_us(0),
          readout_time_us(0) {
        for (int i = 0; i < SPI_NUM_FLOATS; ++i) data[i] = 0.0f;
    }
};

class SPIMasterHandler {
public:
    // device: e.g. "/dev/spidev0.0"
    SPIMasterHandler(const char* device = "/dev/spidev0.0",
                     uint32_t speed_hz = 800000,
                     uint8_t mode = 0,
                     uint8_t bits = 8);

    ~SPIMasterHandler();

    bool has_new_data() const { return new_data_available; }

    void get_received_data(SPIData& out) {
        out = received_data;
        new_data_available = false;
    }

    // Set what we transmit (3 floats). Mirrors "prepare_reply()" idea on the slave.
    void set_tx_floats(float f0, float f1, float f2) {
        tx_data.data[0] = f0;
        tx_data.data[1] = f1;
        tx_data.data[2] = f2;
        prepare_tx();
    }

    // Perform one full-duplex transaction (header+payload+crc).
    // Returns true if a valid frame from the slave was received.
    bool transfer_once();

private:
    int fd;
    uint32_t speed_hz;
    uint8_t mode;
    uint8_t bits;

    uint8_t buffer_rx[SPI_MSG_SIZE];
    uint8_t buffer_tx[SPI_MSG_SIZE];

    SPIData received_data;
    SPIData tx_data;

    bool new_data_available = false;

    steady_clock::time_point time_start;
    steady_clock::time_point time_previous;

    void prepare_tx() {
        buffer_tx[0] = SPI_HEADER_MASTER;
        // Pack floats (little-endian on Pi). memcpy is fine on Pi (LE).
        std::memcpy(&buffer_tx[1], tx_data.data, SPI_NUM_FLOATS * sizeof(float));
        buffer_tx[SPI_MSG_SIZE - 1] = calculate_crc8(buffer_tx, SPI_MSG_SIZE - 1);
    }

    static uint8_t calculate_crc8(const uint8_t* buffer, size_t length) {
        uint8_t crc = 0x00;
        for (size_t i = 0; i < length; ++i) {
            crc ^= buffer[i];
            for (uint8_t j = 0; j < 8; ++j) {
                if (crc & 0x80) crc = (crc << 1) ^ 0x07;
                else            crc <<= 1;
            }
        }
        return crc;
    }

    static bool verify_checksum(const uint8_t* buffer, size_t length, uint8_t expected_crc) {
        return calculate_crc8(buffer, length) == expected_crc;
    }
};

#endif // SPI_MASTER_HANDLER_H_
