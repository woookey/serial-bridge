#include <iostream>
#include <serial2UDP.h>
#include <serial_port.h>
#include <memory>

int main(int argc, char* argv[]) {
    (void)argc;
    (void)argv;
    std::cout << "serial-bridge version " << serial2UDP_VERSION_MAJOR << "."
    << serial2UDP_VERSION_MINOR << std::endl << "------------" << std::endl;

    /// create serial-bridge instance
    std::unique_ptr<serial2UDP::serial_port> uart_to_json =
        std::make_unique<serial2UDP::serial_port>("/dev/ttyUSB0", serial2UDP::BITS_8,
                                                  serial2UDP::PARITY_NONE, serial2UDP::BAUDRATE_115200);
    /// run if initialised successfully
    if (uart_to_json->is_initialised()) {
        uart_to_json->start();
    }
    return 0;
}