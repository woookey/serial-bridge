configure_file(serial2UDP.h.in serial2UDP.h)

target_include_directories(serial2UDP PUBLIC
        "${PROJECT_BINARY_DIR}/inc"
)