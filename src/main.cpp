#include <iostream>
#include <serial_bridge.h>
#include <version.h>
#include <memory>

int main(int argc, char* argv[]) {
    (void)argc;
    (void)argv;
    std::cout << "serial_bridge version " << serial_bridge_VERSION_MAJOR << "."
    << serial_bridge_VERSION_MINOR << std::endl << "------------" << std::endl;

    /// create serial_bridge instance
    std::unique_ptr<serial_bridge::serial_bridge> uart_to_json =
        std::make_unique<serial_bridge::serial_bridge>("/dev/ttyUSB0", serial_bridge::BITS_8,
                                                  serial_bridge::PARITY_NONE, serial_bridge::BAUDRATE_115200);
    /// run if initialised successfully
    if (uart_to_json->is_initialised()) {
        uart_to_json->start();
    }
    return 0;
}