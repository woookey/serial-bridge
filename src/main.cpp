#include <iostream>
#include "serial2UDP.h"
#include "serial_port.h"

int main(int argc, char* argv[]) {
    (void)argv;
    if (argc < 2) {
        std::cout << "serial2UDP Version " << serial2UDP_VERSION_MAJOR << "."
        << serial2UDP_VERSION_MINOR << std::endl;
    }

    std::string port_name = "/dev/ttyUSB0";
    serial2UDP::serial_port UART_to_USB(port_name);

    std::cout << "Starting serial2UDP\n" << "------------\n";
    UART_to_USB.start();
    std::cout << "Exiting serial2UDP\n" << "------------\n";
    return 0;
}