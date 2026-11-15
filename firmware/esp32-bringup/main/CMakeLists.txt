idf_component_register(
    SRCS
        "bu_main.c"         # sequencer: runs every test, prints the summary, idle loop
        "bu_report.c"       # PASS/FAIL/WARN/VISUAL result table
        "bu_swd.c"          # bit-banged SWD master, ADIv6 (RP2350 debug port)
        "bu_oled.c"         # minimal SSD1309 driver: test pattern + result screen
        "bu_leds.c"         # 4x WS2812 chain: chase pattern + result colours
        "bu_test_esp.c"     # chip / flash / PSRAM / USB / WiFi-scan / die temp
        "bu_test_power.c"   # power-path GPIOs, buttons, BOOST_EN, LTC2954 KILL
        "bu_test_i2c.c"     # bus scan + MAX17048 / BQ25180 / RV-3028 identity
        "bu_test_sd.c"      # card detect, mount, write/read/delete, unmount
        "bu_test_rp.c"      # RP2354: SWD attach, SRAM, CHIP_ID, GWSPI link continuity, RUN
        "bu_test_link.c"    # GWSPI link at protocol level: SYNC, GetInfo, diag counters
        # Production link layer, reused verbatim so the bring-up tests the real
        # thing rather than a lookalike. Self-contained: link.h + crc16 + IDF.
        "../../esp32/main/link_spi.c"
        "../../esp32/main/crc16.c"
    # "." first so bu_*.h wins; ../../esp32/main supplies app_config.h (the
    # single source of truth for the pin map) and font8x8.h. Every bring-up
    # file carries the bu_ prefix so nothing collides with the production
    # headers that share that include path.
    INCLUDE_DIRS "." "../../esp32/main"
    PRIV_REQUIRES
        driver          # GPIO, SPI, I2C (i2c_master), sdspi, temperature sensor, usb_serial_jtag
        esp_timer
        esp_psram       # esp_psram_get_size
        spi_flash       # esp_flash_get_size
        fatfs           # esp_vfs_fat_sdspi_mount
        sdmmc
        nvs_flash       # esp_wifi needs NVS initialised
        esp_wifi
        esp_netif
        esp_event
)
