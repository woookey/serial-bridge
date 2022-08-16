#include "serial_port.h"

#include <termios.h>
#include <fcntl.h>
#include <errno.h>
#include <unistd.h>
#include <sys/time.h>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <string.h>
#include <stdio.h>
#include <csignal>
#include <iostream>
#include <thread>

namespace serial2UDP {

serial_port::serial_port(std::string port_name = "/dev/ttyUSB0")
    : port_name_(port_name) {
    memset(&read_buffer_, 0, sizeof(read_buffer_));
    memset(&write_buffer_, 0, sizeof(write_buffer_));
    /// open serial port file descriptor
    serial_port_fd_ = open(port_name_.c_str(), O_RDWR | O_NONBLOCK | O_NOCTTY);
    if (serial_port_fd_ < 0) {
        std::cerr << "[Error] " << errno << " from open: " << strerror(errno) << std::endl;
    }

    /// lock the port
    if (flock(serial_port_fd_, LOCK_EX | LOCK_NB) == -1) {
        std::cerr << "[Error] Failed to lock the file!\n";
    }

    /// start configuring serial port
    struct termios tty_usb;
    if(tcgetattr(serial_port_fd_, &tty_usb) != 0) {
        std::cerr << "[Error] " << errno << " from tcgetattr: " << strerror(errno) << std::endl;
    }

    /// set UART configuration
    cfmakeraw(&tty_usb);
    tty_usb.c_cflag &= ~PARENB;
    tty_usb.c_cflag &= ~CSTOPB;
    tty_usb.c_cflag |= CS7;

    tty_usb.c_cflag &= ~CRTSCTS;
    tty_usb.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty_usb.c_cflag |= CREAD | CLOCAL;

    /// non-canonical mode
    tty_usb.c_lflag &= ~ICANON;
    /// disable echo
    tty_usb.c_lflag &= ~ECHO;
    tty_usb.c_lflag &= ~ECHOE;
    tty_usb.c_lflag &= ~ECHONL;
    tty_usb.c_lflag &= ~ISIG;
    tty_usb.c_lflag &= ~IEXTEN;

    /// disable output processing
    tty_usb.c_oflag &= ~OPOST;

    /// set reading configuration to be blocking with expected bytes to be read
    tty_usb.c_cc[VTIME] = 0;
    tty_usb.c_cc[VMIN] = 0;

    /// set baudrates
    cfsetispeed(&tty_usb, B115200);
    cfsetospeed(&tty_usb, B115200);

    /// set serial port configuration
    if (tcsetattr(serial_port_fd_, TCSANOW, &tty_usb) != 0) {
        std::cerr << "[Error] " << errno << " from tcsetattr: " << strerror(errno) << std::endl;
    }

    /// register SIGINT
    signal(SIGINT, signal_handler);
}

serial_port::~serial_port() {
    std::cout << "Closing serial_port " << port_name_ << std::endl;
    close(serial_port_fd_);
}

void serial_port::start() {
    keep_running = true;

    /// run threads
    read_cmd_thread = std::thread(&serial_port::read_commands, this);
    exchange_data_thread = std::thread(&serial_port::exchange_data, this);

    read_cmd_thread.join();
    exchange_data_thread.join();
}

void serial_port::signal_handler(int signum) {
    std::cout << "Interrupt signal (" << signum << ") received.\n";
    exit(signum);
}

void serial_port::read_commands(void) {
    /// keep reading new commands
    while(keep_running) {
        std::cout << "Provide next cmd (e.g. \"XY\"):\n";
        std::cin.getline(curr_cmd, 100);
        /// verify command
        if ((strcmp(curr_cmd, "XA") == 0)
            || (strcmp(curr_cmd, "FY") == 0)) {
            std::cout << "Next command to send: " << std::string(curr_cmd) << std::endl;
            cmd_to_send = true;
        } else {
            std::cerr << "[Error] Wrong command requested\n";
        }
    }
};

void serial_port::exchange_data(void) {
    fd_set rdset, wrset;
    int r, n;
    int sig_read;

    while (keep_running) {
        struct timeval tv, *ptv;

        ptv = NULL;
        FD_ZERO(&rdset);
        FD_ZERO(&wrset);

        FD_SET(serial_port_fd_, &rdset);

        if(cmd_to_send) {
            FD_SET(serial_port_fd_, &wrset);
        }

        r = select(serial_port_fd_ + 1, &rdset, &wrset, NULL, ptv);
        if ( r < 0 )  {
            if ( errno == EINTR )
                continue;
            else {
                std::cerr << "[Error] select failed " << errno << " " << strerror(errno) << std::endl;
            }

        } else if ( r == 0 ) {
            std::cerr << "[Error] timeout!\n";
        }


        /// reception
        if ( FD_ISSET(serial_port_fd_, &rdset) ) {
            /// read from port byte by byte
            volatile int i =0;
            do {
                n = read(serial_port_fd_, &read_buffer_[i], 1);
                i++;
            } while (n != 0 && errno != EINTR);
            if ( n < 0 ) {
                if ( errno != EAGAIN && errno != EWOULDBLOCK )
                    std::cerr << "[Error] read from port failed: " << strerror(errno) << std::endl;
            } else {
                print_rx_data();
            }

            if (read_buffer_[0] != 0x2A) {
                std::cerr << "[Error] wrong init character\n";
            } else {
                /// send data in JSON over UDP
                (void)read_buffer_;
            }
        }

        /// transmission
        if ( FD_ISSET(serial_port_fd_, &wrset) ) {
            /// write command over port
            n = write(serial_port_fd_, curr_cmd, 2);
            if ( n <= 0 ) {
                std::cerr << "[Error] write to port failed: " << strerror(errno) << std::endl;
            }
            cmd_to_send = false;
        }
    }
}

void serial_port::print_rx_data(void) {
    std::cout << "[RX] Data(0) = " << std::hex << static_cast<int>(read_buffer_[0] & 0x7F) << std::endl;
    std::cout << "[RX] Data(1) = " << std::hex << static_cast<int>(read_buffer_[1] & 0x7F) << std::endl;
    std::cout << "[RX] Data(2) = " << std::hex << static_cast<int>(read_buffer_[2] & 0x7F) << std::endl;
    std::cout << "[RX] Data(3) = " << std::hex << static_cast<int>(read_buffer_[3] & 0x7F) << std::endl;
    std::cout << "[RX] Data(4) = " << std::hex << static_cast<int>(read_buffer_[4] & 0x7F) << std::endl;
    std::cout << "[RX] Data(5) = " << std::hex << static_cast<int>(read_buffer_[5] & 0x7F) << std::endl;
    std::cout << "[RX] Data(6) = " << std::hex << static_cast<int>(read_buffer_[6] & 0x7F) << std::endl;
    std::cout << "[RX] Data(7) = " << std::hex << static_cast<int>(read_buffer_[7] & 0x7F) << std::endl;
    std::cout << "[RX] Data(8) = " << std::hex << static_cast<int>(read_buffer_[8] & 0x7F) << std::endl;
    std::cout << "[RX] Data(9) = " << std::hex << static_cast<int>(read_buffer_[9] & 0x7F) << std::endl;
    std::cout << "[RX] Data(10) = " << std::hex << static_cast<int>(read_buffer_[10] & 0x7F) << std::endl;
    std::cout << "[RX] Data(11) = " << std::hex << static_cast<int>(read_buffer_[11] & 0x7F) << std::endl;
}

}