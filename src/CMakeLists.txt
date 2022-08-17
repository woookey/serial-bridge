add_compile_options(-Wall -Wextra -pedantic -g)

add_executable(serial2UDP
        main.cpp
        serial_port.cpp)

configure_file(../inc/serial2UDP.h.in serial2UDP.h)

target_include_directories(serial2UDP PUBLIC
        ${PROJECT_BINARY_DIR}
        ${CMAKE_SOURCE_DIR}/inc
        ${CMAKE_SOURCE_DIR}/3rdparty
)