#ifndef serial_bridge_H
#define serial_bridge_H

#include <string>
#include <map>
#include <vector>
#include <pthread.h>
#include <thread>
#include <termios.h>
#include <netinet/in.h>

#include <nlohmann/json.hpp>
using json = nlohmann::json;

namespace serial_bridge {

typedef enum : uint32_t {
    BITS_7 = CS7,
    BITS_8 = CS8,
} data_bits_t;

typedef enum : uint8_t {
    PARITY_NONE = 0,
    PARITY_EVEN,
    PARITY_ODD,
} parity_t;

typedef enum : uint32_t {
    BAUDRATE_57600 = B57600,
    BAUDRATE_115200 = B115200,
    BAUDRATE_230400 = B230400,
    BAUDRATE_460800 = B460800,
    BAUDRATE_500000 = B500000,
    BAUDRATE_576000 = B576000,
    BAUDRATE_921600 = B921600,
    BAUDRATE_1000000 = B1000000,
    BAUDRATE_1152000 = B1152000,
    BAUDRATE_1500000 = B1500000,
    BAUDRATE_2000000 = B2000000,
    BAUDRATE_2500000 = B2500000,
    BAUDRATE_3000000 = B3000000,
    BAUDRATE_3500000 = B3500000,
    BAUDRATE_4000000 = B4000000,
} baudrate_t;

class serial_bridge {
private:
    #define ARRAY_MAX_LENGTH 20
    /// serial port data
    std::string port_name_{nullptr};
    int serial_bridge_fd_{0};
    bool keep_running{false};
    bool cmd_to_send{false};
    char curr_cmd[100];
    bool port_lock{false};
    bool init{false};

    /// serial port configuration data
    data_bits_t data_bits_{BITS_7};
    parity_t parity_{PARITY_NONE};
    baudrate_t baudrate_{BAUDRATE_115200};

    const long int nanosec_in_ms{1000000UL};
    const long int nanosec_in_s{1000000000UL};

    const int READ_SIZE{12};
    const int WRITE_SIZE{2};
    uint8_t read_buffer_[ARRAY_MAX_LENGTH];
    uint8_t write_buffer_[ARRAY_MAX_LENGTH];

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
    /// \todo protocol file could be parsed to dynamically assign these values
    const int uid = 0x2A;

public:
    static void signal_handler(int signum);
    serial_bridge(std::string port_name);
    serial_bridge(std::string port_name, data_bits_t data_bits, parity_t parity, baudrate_t baudrate);
    ~serial_bridge();
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