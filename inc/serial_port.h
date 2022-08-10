#ifndef SERIAL_PORT_H
#define SERIAL_PORT_H

#include <string>

namespace serial2UDP {

class serial_port {
private:
    std::string port_name_{nullptr};
    int serial_port_fd_{0};
    bool keep_running{false};

    uint8_t read_buffer_[500];
    uint8_t write_buffer_[2];
public:
    static void signal_handler(int signum);
    serial_port(std::string port_name);
    ~serial_port();
    void start();
    //void comms_loop(serial_port* serial_port_object);

};

}

#endif
