# RAM-only flash stub for the ESP's SWD flasher (flash_stub.c).
# Self-contained pico-sdk project — build separately from the firmware:
#   set PICO_SDK_PATH=<sdk 2.x checkout>
#   cmake -S . -B build && cmake --build build
# -> build/flash_stub.bin: the ESP writes this image into SRAM over the SWD
#    MEM-AP at 0x20000000 and starts it at its ELF entry.
# Regenerate firmware/esp32/main/rp_flash_stub_bin.h from it (see that header).
#
# Keep this SMALL and self-contained: it runs from SRAM with XIP quiesced
# during flash ops, so it must not call into flash-resident code.
cmake_minimum_required(VERSION 3.13)

set(PICO_PLATFORM rp2350-arm-s CACHE STRING "Platform")
set(PICO_BOARD gw_rp2354a CACHE STRING "Board type")
list(APPEND PICO_BOARD_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/../../boards)

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(flash_stub C CXX ASM)
set(CMAKE_C_STANDARD 11)

pico_sdk_init()

add_executable(flash_stub flash_stub.c)

# RAM-only: no flash image, no boot2 — the ESP loads this .bin into SRAM over
# SWD and runs it there (which is the whole point: the flash is the target).
pico_set_binary_type(flash_stub no_flash)

target_link_libraries(flash_stub
    pico_stdlib
    hardware_flash
    hardware_sync
)

pico_add_extra_outputs(flash_stub) # .bin is what the ESP embeds
