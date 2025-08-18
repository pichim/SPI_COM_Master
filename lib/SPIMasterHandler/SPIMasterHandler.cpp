#include "SPIMasterHandler.h"

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <cstdio>

SPIMasterHandler::SPIMasterHandler(const char* device,
                                   uint32_t speed_hz_,
                                   uint8_t mode_,
                                   uint8_t bits_)
    : fd(-1),
      speed_hz(speed_hz_),
      mode(mode_),
      bits(bits_),
      new_data_available(false)
{
    fd = ::open(device, O_RDWR);
    if (fd < 0) throw std::runtime_error("Failed to open SPI device");

    // SPI mode (0)
    if (ioctl(fd, SPI_IOC_WR_MODE, &mode) == -1) throw std::runtime_error("SPI: set mode failed");
    if (ioctl(fd, SPI_IOC_RD_MODE, &mode) == -1) throw std::runtime_error("SPI: get mode failed");

    // Bits per word (8)
    if (ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits) == -1) throw std::runtime_error("SPI: set bits failed");
    if (ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &bits) == -1) throw std::runtime_error("SPI: get bits failed");

    // Speed (Hz)
    if (ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed_hz) == -1) throw std::runtime_error("SPI: set speed failed");
    if (ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed_hz) == -1) throw std::runtime_error("SPI: get speed failed");

    // Initialize TX (mirrors slave's constructor prepare_reply)
    tx_data.data[0] = 42.42f;
    tx_data.data[1] = 98.76f;
    tx_data.data[2] = 11.11f;
    prepare_tx();

    time_start    = steady_clock::now();
    time_previous = time_start;
}

SPIMasterHandler::~SPIMasterHandler() {
    if (fd >= 0) ::close(fd);
}

bool SPIMasterHandler::transfer_once() {
    // Measure processing time similar to slave's on_cs_fall()
    const auto start = steady_clock::now();

    struct spi_ioc_transfer tr{};
    tr.tx_buf        = reinterpret_cast<__u64>(buffer_tx);
    tr.rx_buf        = reinterpret_cast<__u64>(buffer_rx);
    tr.len           = SPI_MSG_SIZE;
    tr.speed_hz      = speed_hz;
    tr.delay_usecs   = 0;
    tr.bits_per_word = bits;
    tr.cs_change     = 0; // keep CS asserted only for this message (default)

    const int ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
    if (ret < 1) {
        // I/O error → count as failed frame
        received_data.failed_count++;
        return false;
    }

    const uint8_t header_rx = buffer_rx[0];
    const uint8_t crc_rx    = buffer_rx[SPI_MSG_SIZE - 1];

    if (verify_checksum(buffer_rx, SPI_MSG_SIZE - 1, crc_rx) &&
        header_rx == SPI_HEADER_SLAVE)
    {
        std::memcpy(received_data.data, &buffer_rx[1], SPI_NUM_FLOATS * sizeof(float));
        received_data.message_count++;

        const auto now    = steady_clock::now();
        const auto dt_us  = duration_cast<microseconds>(now - time_previous).count();
        received_data.last_delta_time_us = static_cast<uint32_t>(dt_us);
        time_previous = now;

        const auto end = steady_clock::now();
        received_data.readout_time_us =
            static_cast<uint32_t>(duration_cast<microseconds>(end - start).count());

        new_data_available = true;
        return true;
    } else {
        received_data.failed_count++;
        new_data_available = false;
        return false;
    }
}
