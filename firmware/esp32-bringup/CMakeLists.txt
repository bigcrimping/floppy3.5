# ESP-IDF v5.2+ — gw-esp32 PCB assembly bring-up firmware (ESP32-S3 side).
#   idf.py set-target esp32s3
#   idf.py build flash monitor
# Pin map is shared with the production bridge: main/ includes ../esp32/main
# for app_config.h, so the bring-up always tests the pins the real firmware
# will use.
cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(gw_esp32_bringup)
