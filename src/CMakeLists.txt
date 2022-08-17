add_compile_options(-Wall -Wextra -pedantic -g)

add_executable(serial_bridge
        main.cpp
        serial_bridge.cpp)

configure_file(../inc/version.h.in serial_bridge.h)

target_include_directories(serial_bridge PUBLIC
        ${PROJECT_BINARY_DIR}
        ${CMAKE_SOURCE_DIR}/inc
        ${CMAKE_SOURCE_DIR}/3rdparty
)