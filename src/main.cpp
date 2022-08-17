// serial-bridge ver. 1.0
// https://github.com/woookey/serial-bridge
//
// MIT License
//
// Copyright (c) 2022 Actuated Robots Ltd., Lukasz Barczyk
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

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