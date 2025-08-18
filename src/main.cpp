// pi@raspberrypi:~/Projects/SPI_COM_Master $ g++ -std=c++17 -O2 -Wall -Wextra -o spi_master main.cpp SPIMasterHandler.cpp
// pi@raspberrypi:~/Projects/SPI_COM_Master $ ./spi_master

#include "SPIMasterHandler.h"
#include <thread>
#include <chrono>
#include <cstdio>

int main() {
    try {
        SPIMasterHandler handler("/dev/spidev0.0", 800000, 0, 8);

        // Keep TX floats constant (matches your Python)
        handler.set_tx_floats(42.42f, 98.76f, 11.11f);

        while (true) {
            handler.transfer_once(); // one full-duplex frame

            if (handler.has_new_data()) {
                SPIData data;
                handler.get_received_data(data);

                // Mirror your mbed printf format/fields
                std::printf("Message: %lu | Delta Time: %lu us | "
                            "Received: [%.2f, %.2f, %.2f] | "
                            "Header: 0x%02X | Failed: %lu | "
                            "Readout Time: %lu us\n",
                    static_cast<unsigned long>(data.message_count),
                    static_cast<unsigned long>(data.last_delta_time_us),
                    data.data[0], data.data[1], data.data[2],
                    SPI_HEADER_SLAVE,
                    static_cast<unsigned long>(data.failed_count),
                    static_cast<unsigned long>(data.readout_time_us));
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(20)); // ~50 Hz
        }
    } catch (const std::exception& e) {
        std::fprintf(stderr, "Error: %s\n", e.what());
        return 1;
    }
}
