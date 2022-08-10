#include "serial_port.h"

#include <termios.h>
#include <fcntl.h>
#include <errno.h>
#include <unistd.h>
#include <time.h>
#include <sys/time.h>

#include <string.h>
#include <stdio.h>
#include <csignal>
#include <iostream>
#include <thread>

namespace serial2UDP {

serial_port::serial_port(std::string port_name = "/dev/ttyUSB0")
    : port_name_(port_name) {
    memset(&read_buffer_, 0, sizeof(read_buffer_));
    /// open serial port file descriptor
    serial_port_fd_ = open(port_name_.c_str(), O_RDWR);

    if (serial_port_fd_ < 0) {
        printf("Error %i from open: %s\n", errno, strerror(errno));
    }

    /// start configuring serial port
    struct termios tty_usb;
    if(tcgetattr(serial_port_fd_, &tty_usb) != 0) {
        printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
    }
    tty_usb.c_cflag &= ~PARENB;
    tty_usb.c_cflag &= ~CSTOPB;
    tty_usb.c_cflag |= CS7;
    tty_usb.c_cflag &= ~CRTSCTS;
    tty_usb.c_cflag |= CREAD | CLOCAL;


    tty_usb.c_lflag &= ~ICANON;
    tty_usb.c_lflag &= ~ECHO;
    tty_usb.c_lflag &= ~ISIG;

    tty_usb.c_cc[VTIME] = 10;
    tty_usb.c_cc[VMIN] = 0;

    cfsetispeed(&tty_usb, B115200);
    cfsetospeed(&tty_usb, B115200);

    if (tcsetattr(serial_port_fd_, TCSANOW, &tty_usb) != 0) {
        printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
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

    write_buffer_[0] = 'X';
    write_buffer_[1] = 'A';

    auto serial_comms_loop = [](serial_port* serial_port_object) {
        int num_bytes;
        const long int nanosec_in_ms = 1000000UL;
        const long int nanosec_in_s = 1000000000UL;
        struct timespec period = {0, nanosec_in_ms*10UL};
        struct timespec activate_time;
        int err = clock_gettime(CLOCK_MONOTONIC, &activate_time);

        while(serial_port_object->keep_running) {
            /// wait until activation time
            do {
                err = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &activate_time, NULL);
            } while ((err != 0) && (errno == EINTR));

            /// process communication
            write(serial_port_object->serial_port_fd_, serial_port_object->write_buffer_, 2);

/*            num_bytes = read(serial_port_object->serial_port_fd_, serial_port_object->read_buffer_, sizeof(serial_port_object->read_buffer_));
            if (num_bytes <= 0) {
                std::cerr << "[Error] No incoming data!\n";
            }*/

            /// add waiting time until next activation
            if (activate_time.tv_nsec + period.tv_nsec > nanosec_in_s) {
                activate_time.tv_sec += 1;
                activate_time.tv_nsec += period.tv_nsec;
                activate_time.tv_nsec -= nanosec_in_s;
            } else {
                activate_time.tv_nsec += period.tv_nsec;
            }
        }
    };
    auto simple_serial_comms_loop = [](serial_port* serial_port_object) {
        int num_bytes;
        const long int nanosec_in_ms = 1000000UL;
        const long int nanosec_in_s = 1000000000UL;
        struct timespec period = {0, nanosec_in_ms*10UL};
        struct timespec activate_time;
        int err = clock_gettime(CLOCK_MONOTONIC, &activate_time);

        while(1) {

            /// process communication
            write(serial_port_object->serial_port_fd_, serial_port_object->write_buffer_, 2);

/*            num_bytes = read(serial_port_object->serial_port_fd_, serial_port_object->read_buffer_, sizeof(serial_port_object->read_buffer_));
            if (num_bytes <= 0) {
                std::cerr << "[Error] No incoming data!\n";
            }*/
///900000000UL
            std::this_thread::sleep_for(std::chrono::nanoseconds(900000000UL));
        }
    };
    std::thread data_rx_thread(simple_serial_comms_loop, this);
    data_rx_thread.join();
/*    const long int nanosec_in_ms = 1000000UL;
    const long int nanosec_in_s = 1000000000UL;
    struct timespec period = {0, nanosec_in_ms*100UL};
    struct timespec activate_time;
    int err = clock_gettime(CLOCK_MONOTONIC, &activate_time);

    while(keep_running) {
        /// wait until activation time
        do {
            err = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &activate_time, NULL);
        } while ((err != 0) && (errno == EINTR));

        /// process communication
        write(serial_port_fd_, write_buffer_, 2);
        num_bytes = read(serial_port_fd_, read_buffer_, sizeof(read_buffer_));
        if (num_bytes <= 0) {
            std::cerr << "[Error] No incoming data!\n";
        }

        /// add waiting time until next activation
        if (activate_time.tv_nsec + period.tv_nsec > nanosec_in_s) {
            activate_time.tv_sec += 1;
            activate_time.tv_nsec += period.tv_nsec;
            activate_time.tv_nsec -= nanosec_in_s;
        } else {
            activate_time.tv_nsec += period.tv_nsec;
        }
    }*/
}

void serial_port::signal_handler(int signum) {
    std::cout << "Interrupt signal (" << signum << ") received.\n";
    //close(serial_port_fd_);
    exit(signum);
}

}