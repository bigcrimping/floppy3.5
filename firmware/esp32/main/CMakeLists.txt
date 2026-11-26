idf_component_register(
    SRCS
        "app_main.c"
        "gw_client.c"
        "link_spi.c"
        "flux_decode.c"
        "mfm_decode.c"
        "mfm_encode.c"
        "crc16.c"
        "disk_image.c"
        "fat_scan.c"
        "fat12_diskio.c"
        "provision.c"
        "web.c"
        "web_util.c"
        "netinfo.c"
        "storage.c"
        "sd_index.c"    # SHA-256 content index for SD dedup
        "display.c"
        "ui.c"
        "op_queue.c"    # order of the ops waiting for the gw task
        "leds.c"        # 4x WS2812 status strip (progress / state / battery)
        "power.c"
        "wallclock.c"   # RV-3028-C7 wall clock (file timestamps + GUI clock)
        "serialno.c"    # board serial number in the RV-3028 user EEPROM
        "temps.c"       # ESP die / RP die / battery-zone temperatures
        "rp_swd.c"      # bit-banged SWD master (controller flashing)
        "rp_flash.c"    # SWD + RAM-stub flasher, /api/rpflash
        "ota.c"         # bridge self-update over WiFi, /api/ota
        "wifi_pwr.c"    # WiFi TX/radio policy during drive sessions
        "lastoff.c"     # shutdown breadcrumb in NVS (lastoff in /api/status)
    INCLUDE_DIRS "."
    EMBED_TXTFILES "www/index.html" "www/setup.html"
    PRIV_REQUIRES
        driver          # SPI, GPIO
        esp_timer       # esp_timer_get_time (GW_PROFILE dump timing)
        esp_http_server
        esp_wifi
        esp_netif
        esp_event
        mdns
        fatfs           # RAM FAT12 image + SD (esp_vfs_fat_sdspi_mount)
        sdmmc           # SD card protocol layer (sdspi host lives in `driver`)
        nvs_flash
        app_update      # esp_ota_* — bridge self-update (ota.c)
        esp_partition   # OTA slot lookup
        mbedtls         # SHA-256 (hardware-accelerated on the S3) — sd_index.c
)

# The per-transition decode and encode loops run ~110k times a track: build
# them optimised even in a debug (-Og) build. Later flags win.
set_source_files_properties(mfm_decode.c flux_decode.c mfm_encode.c crc16.c link_spi.c
    PROPERTIES COMPILE_OPTIONS "-O2")
