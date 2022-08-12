#ifndef SERIAL_PORT_H
#define SERIAL_PORT_H

#include <string>
#include <map>
#include <vector>
#include <pthread.h>

namespace serial2UDP {

class serial_port {
private:
    std::string port_name_{nullptr};
    int serial_port_fd_{0};
    bool keep_running{false};
    bool cmd_to_sent{false};
    char curr_cmd[100];
    bool port_lock{false};

    const long int nanosec_in_ms{1000000UL};
    const long int nanosec_in_s{1000000000UL};

    const int READ_SIZE{12};
    const int WRITE_SIZE{2};

    uint8_t read_buffer_[12];
    uint8_t write_buffer_[2];

    pthread_mutex_t mtx = PTHREAD_MUTEX_INITIALIZER;

    /// commands map is filled during initialisation
    std::map<std::string, std::vector<uint8_t>> cmd_storage();
public:
    static void signal_handler(int signum);
    serial_port(std::string port_name);
    ~serial_port();
    void start();
};

}

#endif
