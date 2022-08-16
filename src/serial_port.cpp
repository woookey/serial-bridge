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
        volatile int sig_write;
        volatile int sig_read;

        /// keep reading new commands
        while(serial_port_object->keep_running) {
            std::cout << "Provide next cmd (e.g. \"XY\"):\n";
            std::cin.getline(serial_port_object->curr_cmd, 100);
            /// verify command
            if ((strcmp(serial_port_object->curr_cmd, "XA") == 0)
            || (strcmp(serial_port_object->curr_cmd, "FY") == 0)) {

                /// wait until no data is queued to be read
                do {
                    ioctl(serial_port_object->serial_port_fd_, FIONREAD, &sig_read);
                } while(sig_read != 0);

                pthread_mutex_lock(&serial_port_object->mtx);

                ioctl(serial_port_object->serial_port_fd_, TIOCOUTQ, &sig_write);
                ioctl(serial_port_object->serial_port_fd_, FIONREAD, &sig_read);
                std::cout << "TX write_sig_before_flush = " << sig_write << ", read_sig_before_flush = " << sig_read << std::endl;

                if(tcflush(serial_port_object->serial_port_fd_, TCOFLUSH) != 0) {
                    std::cerr << "Tx queue cannot flush\n";
                }
/*                if (sig_read != 0) {
                    if(tcflush(serial_port_object->serial_port_fd_, TCIFLUSH) != 0) {
                        std::cerr << "Tx RX queue cannot flush\n";
                    }
                }*/

                ioctl(serial_port_object->serial_port_fd_, TIOCOUTQ, &sig_write);
                ioctl(serial_port_object->serial_port_fd_, FIONREAD, &sig_read);
                std::cout << "TX write_sig_before_write = " << sig_write << ", read_sig_before_write = " << sig_read << std::endl;
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
                ioctl(serial_port_object->serial_port_fd_, TIOCOUTQ, &sig_write);
                ioctl(serial_port_object->serial_port_fd_, FIONREAD, &sig_read);
                std::cout << "TX write_sig_after_push = " << sig_write << ", read_sig_after_push = " << sig_read << std::endl;
                //std::this_thread::sleep_for(std::chrono::nanoseconds(serial_port_object->nanosec_in_ms*2UL));


                std::this_thread::sleep_for(std::chrono::nanoseconds(serial_port_object->nanosec_in_ms*10UL));
/*                if(tcdrain(serial_port_object->serial_port_fd_) == -1) {
                    std::cerr << "[Error] Tx not finished" << errno << std::endl;
                }*/
                if(tcflush(serial_port_object->serial_port_fd_, TCOFLUSH) != 0) {
                    std::cerr << "Tx queue cannot flush\n";
                }
                pthread_mutex_unlock(&serial_port_object->mtx);

                ioctl(serial_port_object->serial_port_fd_, TIOCOUTQ, &sig_write);
                ioctl(serial_port_object->serial_port_fd_, FIONREAD, &sig_read);
                std::cout << "TX write_sig_after_pseudo_drain = " << sig_write << ", read_sig_after_pseudo_drain = " << sig_read << std::endl;


            } else {
                std::cerr << "[Error] Wrong command requested\n";
            }
        }
    };

    auto serial_comms_loop = [](serial_port* serial_port_object) {
        int num_bytes;
        int sig = 0;
        int sig_write;

        while(serial_port_object->keep_running) {
            /// process rx
//            pthread_mutex_lock(&serial_port_object->mtx);

            /// check if RX queue contains any data to be read
            ioctl(serial_port_object->serial_port_fd_, FIONREAD, &sig);
            ioctl(serial_port_object->serial_port_fd_, TIOCOUTQ, &sig_write);
            std::cout << "RX read_sig_before = " << std::dec << sig << ", write_sig_before = " << sig_write << std::endl;
            if (sig > 0 ) {
                /// verify unique ID
                num_bytes = read(serial_port_object->serial_port_fd_, &serial_port_object->read_buffer_[0], 1);
                if (num_bytes == 1 && serial_port_object->read_buffer_[0] == 0x2A) {
                    /// read the rest of bytes
                    num_bytes = read(serial_port_object->serial_port_fd_, &serial_port_object->read_buffer_[1], 11);
/*                    std::cout << "[RX] Data(0) = " << std::hex << static_cast<int>(serial_port_object->read_buffer_[0] & 0x7F) << std::endl;
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
                    std::cout << "[RX] Data(11) = " << std::hex << static_cast<int>(serial_port_object->read_buffer_[11] & 0x7F) << std::endl;*/
                } else {
                    if(tcflush(serial_port_object->serial_port_fd_, TCIFLUSH) != 0) {
                        std::cerr << "Rx queue cannot flush\n";
                    } else {
                        std::cout << "Rx flushing\n";
                    }
                }
            }
            ioctl(serial_port_object->serial_port_fd_, FIONREAD, &sig);
            ioctl(serial_port_object->serial_port_fd_, TIOCOUTQ, &sig_write);
            std::cout << "RX read_sig_after = " << std::dec << sig << ", write_sig_after = " << sig_write << std::endl;
//            pthread_mutex_unlock(&serial_port_object->mtx);

            /// send data
            if(serial_port_object->cmd_to_send) {
                /// send data
                std::cout << "[LOOP] data to be sent\n";
                write(serial_port_object->serial_port_fd_, serial_port_object->curr_cmd, 2);
                serial_port_object->cmd_to_send = false;
            }

            /// \todo timer could be more precise to safe CPU time
            std::this_thread::sleep_for(std::chrono::nanoseconds(serial_port_object->nanosec_in_ms*90UL));
        }
    };

    /// run threads
    //std::thread data_rx_thread(serial_comms_loop, this);
    read_cmd_thread = std::thread(&serial_port::read_commands, this);
    exchange_data_thread = std::thread(&serial_port::exchange_data, this);

    //data_rx_thread.join();
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

/*
    state = ST_TRANSPARENT;
    if ( ! opts.exit )
        stdin_closed = 0;
    else
        stdin_closed = 1;
*/

    while (keep_running) {
        struct timeval tv, *ptv;

        ptv = NULL;
        FD_ZERO(&rdset);
        FD_ZERO(&wrset);
        //if ( ! stdin_closed ) FD_SET(STI, &rdset);
        //if ( ! opts.exit ) FD_SET(serial_port_fd_, &rdset);
        //FD_SET(serial_port_fd_, &rdset);
        //ioctl(serial_port_fd_, FIONREAD, &sig_read);
        //if (sig_read != 0) {
            //std::cout << "sig read = " << std::dec << sig_read << std::endl;
            FD_SET(serial_port_fd_, &rdset);
        //}

        if(cmd_to_send) {
            FD_SET(serial_port_fd_, &wrset);
        }/* else {
            if ( opts.exit_after >= 0 ) {
                msec2tv(&tv, opts.exit_after);
                ptv = &tv;
            } else if ( stdin_closed ) {
                *//* stdin closed, output queue empty, and no
                   idle timeout: Exit. *//*
                return LE_STDIN;
            }
        }*/

        //std::cout << "r = " << r << std::endl;
        r = select(serial_port_fd_ + 1, &rdset, &wrset, NULL, ptv);
        if ( r < 0 )  {
            if ( errno == EINTR )
                continue;
            else
                //fatal("select failed: %d : %s", errno, strerror(errno));
                printf("select failed: %d : %s\n", errno, strerror(errno));
        } else if ( r == 0 ) {
            std::cerr << "Timeout!\n";
        } else {
            //std::cout << "r = " << r << std::endl;
        }


        if ( FD_ISSET(serial_port_fd_, &rdset) ) {
            /// read from port
            volatile int i =0;
            do {
                n = read(serial_port_fd_, &read_buffer_[i], 1);
                //std::cout << std::dec << i << ": n[" << n << "] = " << std::hex << static_cast<int>(read_buffer_[i] & 0x7F) << std::endl;
                i++;
            } while (n != 0 && errno != EINTR);
            if (n == 0) {
                //fatal("read zero bytes from port");
                //std::cerr << "read zero bytes from port\n";
            } else if ( n < 0 ) {
                if ( errno != EAGAIN && errno != EWOULDBLOCK )
                    //fatal("read from port failed: %s", strerror(errno));
                    std::cerr << "read from port failed: " << strerror(errno) << std::endl;
            } else {
                //std::cout << "received data\n";
                print_rx_data();
            }
            if (read_buffer_[0] != 0x2A) {
                std::cerr << "Wrong init character\n";
            }
            //FD_ZERO(&rdset);


/*            do {
                n = read(serial_port_fd_, &read_buffer_, 1);
            } while (n < 0 && errno == EINTR);
            if (n == 0) {
                //fatal("read zero bytes from port");
                std::cerr << "read zero bytes from port\n";
            } else if ( n < 0 ) {
                if ( errno != EAGAIN && errno != EWOULDBLOCK )
                    //fatal("read from port failed: %s", strerror(errno));
                    std::cerr << "read from port failed: " << strerror(errno) << std::endl;
            } else {
                //std::cout << "received data\n";
                print_rx_data();
            }
            FD_ZERO(&rdset);*/
        }

        if ( FD_ISSET(serial_port_fd_, &wrset) ) {
            //volatile int i = 0;
            //for (i = 0; i < 10; i++) {
                n = write(serial_port_fd_, curr_cmd, 2);
             //   std::cout << std::dec << i << ": write[" << n << "]\n";
           // }
            /*do {
                //n = write(serial_port_fd_, tty_q.buff, sz);
                n = write(serial_port_fd_, curr_cmd, 1);
                //std::cout << "write[" << std::dec << i << "]\n";
            } while ( n < 0 && errno == EINTR );*/
            if ( n <= 0 ) {
                //fatal("write to port failed: %s", strerror(errno));
                std::cerr << "write to port failed: " << strerror(errno) << std::endl;
            } else {
                std::cout << "characters sent: " << std::dec << n << std::endl;
            }
            cmd_to_send = false;

/*            if ( opts.lecho && opts.log_filename )
                if ( writen_ni(log_fd, tty_q.buff, n) < n )
                    fatal("write to logfile failed: %s", strerror(errno));
            memmove(tty_q.buff, tty_q.buff + n, tty_q.len - n);
            tty_q.len -= n;*/
        }
    }
    //return LE_SIGNAL;
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