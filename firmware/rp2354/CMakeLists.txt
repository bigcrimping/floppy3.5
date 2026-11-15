# pico-sdk 2.x build (platformio.ini is an untested alternative).
#
#   set PICO_SDK_PATH=<sdk 2.x checkout>
#   cmake -S . -B build && cmake --build build
#
# Output: build/gw_rp2354.uf2 (BOOTSEL drag-drop / picotool) plus .bin for
# the ESP flasher (docs/bringup.md §flashing).
cmake_minimum_required(VERSION 3.13)

# RP2350, ARM secure-only build (the SDK default for Cortex-M33 parts).
set(PICO_PLATFORM rp2350-arm-s CACHE STRING "Platform")
# Custom board: RP2350A variant + 2 MB in-package flash (boards/gw_rp2354a.h).
set(PICO_BOARD gw_rp2354a CACHE STRING "Board type")
list(APPEND PICO_BOARD_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/boards)

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(gw_rp2354 C CXX ASM)
set(CMAKE_C_STANDARD 11)

pico_sdk_init()

file(GLOB GW_SOURCES CONFIGURE_DEPENDS src/*.c)
add_executable(gw_rp2354 ${GW_SOURCES})

pico_generate_pio_header(gw_rp2354 ${CMAKE_CURRENT_SOURCE_DIR}/src/flux_read.pio)
pico_generate_pio_header(gw_rp2354 ${CMAKE_CURRENT_SOURCE_DIR}/src/flux_write.pio)

target_include_directories(gw_rp2354 PRIVATE
    src                      # tusb_config.h must be visible to TinyUSB
)

# Build id for diag_version() / the CDC1 'v' banner.
execute_process(
    COMMAND git describe --always --dirty
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    OUTPUT_VARIABLE GW_BUILD_ID
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
)
if(NOT GW_BUILD_ID)
    set(GW_BUILD_ID "dev")
endif()
target_compile_definitions(gw_rp2354 PRIVATE
    GW_BUILD_ID="${GW_BUILD_ID}"
    CFG_TUSB_OS=OPT_OS_PICO
)

target_link_libraries(gw_rp2354
    pico_stdlib
    pico_unique_id           # USB serial string
    pico_bootrom             # 1200-baud-touch -> reset_usb_boot
    tinyusb_device           # raw device stack; descriptors are first-party
    hardware_spi
    hardware_dma
    hardware_pio
    hardware_irq
    hardware_adc             # on-chip die temperature (GetInfo 0x89)
)

# No pico_enable_stdio_usb: it would fight usb_descriptors.c for the CDCs.

pico_add_extra_outputs(gw_rp2354)   # .uf2 + .bin for BOOTSEL / ESP flashing
