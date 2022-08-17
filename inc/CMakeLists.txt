configure_file(version.h.in version.h)

target_include_directories(serial_bridge PUBLIC
        "${PROJECT_BINARY_DIR}/inc"
)