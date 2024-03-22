add_library( CC1101
    CC1101.h
    CC1101.cpp
    )
    
target_include_directories(CC1101 PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")

target_link_libraries(CC1101
    pico_stdlib hardware_spi
)