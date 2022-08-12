#include "serial_port.h"

#include <termios.h>
#include <fcntl.h>
#include <errno.h>
#include <unistd.h>
//#include <time.h>

#include <sys/time.h>
#include <sys/file.h>
#include <sys/ioctl.h>

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
    serial_port_fd_ = open(port_name_.c_str(), O_RDWR | O_NOCTTY);
    if (serial_port_fd_ < 0) {
        printf("Error %i from open: %s\n", errno, strerror(errno));
    }

    /// lock the port
    if (flock(serial_port_fd_, LOCK_EX | LOCK_NB) == -1) {
        std::cerr << "[Error] Failed to lock the file!\n";
    }

    /// start configuring serial port
    struct termios tty_usb;
    if(tcgetattr(serial_port_fd_, &tty_usb) != 0) {
        printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
    }

    /// set UART configuration
    tty_usb.c_cflag &= ~PARENB;
    tty_usb.c_cflag &= ~CSTOPB;
    tty_usb.c_cflag |= CS7;
    tty_usb.c_cflag &= ~CRTSCTS;
    tty_usb.c_cflag |= CREAD | CLOCAL;

    /// non-canonical mode
    tty_usb.c_lflag &= ~ICANON;
    /// disable echo
    tty_usb.c_lflag &= ~ECHO;
    tty_usb.c_lflag &= ~ECHOE;
    tty_usb.c_lflag &= ~ECHONL;
    tty_usb.c_lflag &= ~ISIG;

    /// set reading configuration to be blocking with expected bytes to be read
    tty_usb.c_cc[VTIME] = 0;
    tty_usb.c_cc[VMIN] = 12;

    /// set baudrates
    cfsetispeed(&tty_usb, B115200);
    cfsetospeed(&tty_usb, B115200);

    /// set serial port configuration
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

    auto cmd_process = [](serial_port* serial_port_object) {
        int num_bytes;

        /// keep reading new commands
        while(serial_port_object->keep_running) {
            std::cout << "Provide next cmd (e.g. XY):\n";
            std::cin.getline(serial_port_object->curr_cmd, 100);
            /// verify command
            if ((strcmp(serial_port_object->curr_cmd, "XA") == 0)
            || (strcmp(serial_port_object->curr_cmd, "FY") == 0)) {
                pthread_mutex_lock(&serial_port_object->mtx);

                if(tcflush(serial_port_object->serial_port_fd_, TCOFLUSH) != 0) {
                    std::cerr << "Tx queue cannot flush\n";
                }

                /// send command
                num_bytes = write(serial_port_object->serial_port_fd_, serial_port_object->curr_cmd, 2);
                if (num_bytes < 0) {
                    std::cerr << "[Error] Tx write error\n";
                } else if (num_bytes != 2) {
                    std::cerr << "[Error] Incorrect number of bytes sent\n";
                } else {
                    std::cout << "Command queued for sending\n";
                }

                // works just with a write above, no need to add lines below (I hope!)
                //ioctl(serial_port_object->serial_port_fd_, FIONREAD, &sig);
                //std::cout << "write_sig_after_push = " << sig << std::endl;
                //std::this_thread::sleep_for(std::chrono::nanoseconds(serial_port_object->nanosec_in_ms*10UL));

                if(tcdrain(serial_port_object->serial_port_fd_) == -1) {
                    std::cout << "[Error] Tx not finished" << errno << std::endl;
                }

                pthread_mutex_unlock(&serial_port_object->mtx);
            } else {
                std::cout << "[Error] Wrong command requested\n";
            }
        }
    };

    auto serial_comms_loop = [](serial_port* serial_port_object) {
        int num_bytes;
        int sig = 0;
        /*struct timespec period = {0, nanosec_in_ms*10UL};
        struct timespec activate_time;
        int err = clock_gettime(CLOCK_MONOTONIC, &activate_time);*/

        while(serial_port_object->keep_running) {
            /// wait until activation time
/*            do {
                err = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &activate_time, NULL);
            } while ((err != 0) && (errno == EINTR));*/

            /// process rx
            pthread_mutex_lock(&serial_port_object->mtx);

            /// check if RX queue contains any data to be read
            ioctl(serial_port_object->serial_port_fd_, FIONREAD, &sig);
            if (sig > 0 ) {
                /// verify unique ID
                num_bytes = read(serial_port_object->serial_port_fd_, &serial_port_object->read_buffer_[0], 1);
                if (num_bytes == 1 && serial_port_object->read_buffer_[0] == 0x2A) {
                    /// read the rest of bytes
                    num_bytes = read(serial_port_object->serial_port_fd_, &serial_port_object->read_buffer_[1], 11);
                    std::cout << "[RX] Data(0) = " << std::hex << static_cast<int>(serial_port_object->read_buffer_[0] & 0x7F) << std::endl;
                    std::cout << "[RX] Data(1) = " << std::hex << static_cast<int>(serial_port_object->read_buffer_[1] & 0x7F) << std::endl;
                    std::cout << "[RX] Data(2) = " << std::hex << static_cast<int>(serial_port_object->read_buffer_[2] & 0x7F) << std::endl;
                    std::cout << "[RX] Data(3) = " << std::hex << static_cast<int>(serial_port_object->read_buffer_[3] & 0x7F) << std::endl;
                    std::cout << "[RX] Data(4) = " << std::hex << static_cast<int>(serial_port_object->read_buffer_[4] & 0x7F) << std::endl;
                    std::cout << "[RX] Data(5) = " << std::hex << static_cast<int>(serial_port_object->read_buffer_[5] & 0x7F) << std::endl;
                    std::cout << "[RX] Data(6) = " << std::hex << static_cast<int>(serial_port_object->read_buffer_[6] & 0x7F) << std::endl;
                    std::cout << "[RX] Data(7) = " << std::hex << static_cast<int>(serial_port_object->read_buffer_[7] & 0x7F) << std::endl;
                    std::cout << "[RX] Data(8) = " << std::hex << static_cast<int>(serial_port_object->read_buffer_[8] & 0x7F) << std::endl;
                    std::cout << "[RX] Data(9) = " << std::hex << static_cast<int>(serial_port_object->read_buffer_[9] & 0x7F) << std::endl;
                    std::cout << "[RX] Data(10) = " << std::hex << static_cast<int>(serial_port_object->read_buffer_[10] & 0x7F) << std::endl;
                    std::cout << "[RX] Data(11) = " << std::hex << static_cast<int>(serial_port_object->read_buffer_[11] & 0x7F) << std::endl;
                } else {
                    if(tcflush(serial_port_object->serial_port_fd_, TCIFLUSH) != 0) {
                        std::cerr << "TCIflush error\n";
                    }
                }

            }
            pthread_mutex_unlock(&serial_port_object->mtx);
            std::this_thread::sleep_for(std::chrono::nanoseconds(serial_port_object->nanosec_in_ms*90UL));

            /// add waiting time until next activation
/*            if (activate_time.tv_nsec + period.tv_nsec > nanosec_in_s) {
                activate_time.tv_sec += 1UL;
                activate_time.tv_nsec += period.tv_nsec;
                activate_time.tv_nsec -= nanosec_in_s;
            } else {
                activate_time.tv_nsec += period.tv_nsec;
            }*/
            //i++;
            //std::this_thread::sleep_for(std::chrono::nanoseconds(nanosec_in_ms*9UL));
        }
    };

    /// run threads
    std::thread data_rx_thread(serial_comms_loop, this);
    std::thread cmd_thread(cmd_process, this);
    cmd_thread.join();
    data_rx_thread.join();
}

void serial_port::signal_handler(int signum) {
    std::cout << "Interrupt signal (" << signum << ") received.\n";
    //close(serial_port_fd_);
    exit(signum);
}

}