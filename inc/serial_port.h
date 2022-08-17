#ifndef SERIAL_PORT_H
#define SERIAL_PORT_H

#include <string>
#include <map>
#include <vector>
#include <pthread.h>
#include <thread>
#include <termios.h>
#include <netinet/in.h>

#include <nlohmann/json.hpp>
using json = nlohmann::json;

namespace serial2UDP {

typedef enum : uint32_t {
    BITS_7 = CS7,
    BITS_8 = CS8,
} data_bits_t;

typedef enum : uint8_t {
    PARITY_NONE = 0,
    PARITY_EVEN,
    PARITY_ODD,
} parity_t;

class serial_port {
private:
    /// serial port data
    std::string port_name_{nullptr};
    int serial_port_fd_{0};
    bool keep_running{false};
    bool cmd_to_send{false};
    char curr_cmd[100];
    bool port_lock{false};
    bool init{false};

    /// serial port configuration data
    data_bits_t data_bits;
    //baudrate_t baudrate;

    const long int nanosec_in_ms{1000000UL};
    const long int nanosec_in_s{1000000000UL};

    const int READ_SIZE{12};
    const int WRITE_SIZE{2};

    uint8_t read_buffer_[12];
    uint8_t write_buffer_[2];

    pthread_mutex_t mtx = PTHREAD_MUTEX_INITIALIZER;

    std::thread read_cmd_thread;
    std::thread exchange_data_thread;

    /// commands map is filled during initialisation
    std::map<std::string, std::vector<uint8_t>> cmd_storage();

    /// networking data
    const std::string udp_hostname{"127.0.0.1"};
    uint16_t udp_port = 9870;
    int sock;
    sockaddr_in destination;

    /// protocol constants
    const int uid = 0x2A;

public:
    static void signal_handler(int signum);
    serial_port(std::string port_name);
    ~serial_port();
    void start();

    bool is_initialised(void) {return init;}

    /// threads
    void read_commands(void);
    void exchange_data(void);

    /// JSON handling
    void update_telemetry_in_json(json& telemetry);
};

}

#endif
