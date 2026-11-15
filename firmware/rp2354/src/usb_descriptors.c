// SPDX-License-Identifier: Unlicense
// Composite device, 2x CDC-ACM.
//
// Uses the pico-sdk default VID:PID, not Greaseweazle's 1209:4D69, so gw.exe
// won't auto-detect it: pass the port, e.g. `gw info --device COM7`.
#include "tusb.h"

#include "pico/bootrom.h"
#include "pico/unique_id.h"

#define USB_VID 0x2E8A
#define USB_PID 0x000A
#define USB_BCD 0x0200

enum {
  ITF_NUM_CDC_0 = 0, // Greaseweazle protocol
  ITF_NUM_CDC_0_DATA,
  ITF_NUM_CDC_1, // debug console
  ITF_NUM_CDC_1_DATA,
  ITF_NUM_TOTAL
};

enum {
  STRID_LANGID = 0,
  STRID_MANUFACTURER,
  STRID_PRODUCT,
  STRID_SERIAL,
  STRID_CDC_0,
  STRID_CDC_1,
};

// Misc/IAD class: required for a composite with more than one CDC.
static const tusb_desc_device_t desc_device = {
    .bLength = sizeof(tusb_desc_device_t),
    .bDescriptorType = TUSB_DESC_DEVICE,
    .bcdUSB = USB_BCD,
    .bDeviceClass = TUSB_CLASS_MISC,
    .bDeviceSubClass = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor = USB_VID,
    .idProduct = USB_PID,
    .bcdDevice = 0x0100,
    .iManufacturer = STRID_MANUFACTURER,
    .iProduct = STRID_PRODUCT,
    .iSerialNumber = STRID_SERIAL,
    .bNumConfigurations = 1,
};

const uint8_t *tud_descriptor_device_cb(void) {
  return (const uint8_t *)&desc_device;
}

#define CONFIG_TOTAL_LEN (TUD_CONFIG_DESC_LEN + 2 * TUD_CDC_DESC_LEN)

#define EPNUM_CDC_0_NOTIF 0x81
#define EPNUM_CDC_0_OUT 0x02
#define EPNUM_CDC_0_IN 0x82
#define EPNUM_CDC_1_NOTIF 0x83
#define EPNUM_CDC_1_OUT 0x04
#define EPNUM_CDC_1_IN 0x84

static const uint8_t desc_configuration[] = {
    // 100 mA: the drive is powered from the ESP's boost rail, not VBUS
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, 0, 100),
    TUD_CDC_DESCRIPTOR(ITF_NUM_CDC_0, STRID_CDC_0, EPNUM_CDC_0_NOTIF, 8,
                       EPNUM_CDC_0_OUT, EPNUM_CDC_0_IN, 64),
    TUD_CDC_DESCRIPTOR(ITF_NUM_CDC_1, STRID_CDC_1, EPNUM_CDC_1_NOTIF, 8,
                       EPNUM_CDC_1_OUT, EPNUM_CDC_1_IN, 64),
};

const uint8_t *tud_descriptor_configuration_cb(uint8_t index) {
  (void)index;
  return desc_configuration;
}

static const char *const string_desc_arr[] = {
    [STRID_MANUFACTURER] = "rp-floppy",
    [STRID_PRODUCT] = "gw-rp2354 floppy bridge",
    [STRID_CDC_0] = "gw-rp2354 GW protocol",
    [STRID_CDC_1] = "gw-rp2354 debug console",
};

const uint16_t *tud_descriptor_string_cb(uint8_t index, uint16_t langid) {
  (void)langid;
  static uint16_t desc_str[34];
  uint8_t len;

  if (index == STRID_LANGID) {
    desc_str[1] = 0x0409; // English (US)
    len = 1;
  } else if (index == STRID_SERIAL) {
    // Per-chip serial so each board keeps its own COM port.
    char serial[2 * PICO_UNIQUE_BOARD_ID_SIZE_BYTES + 1];
    pico_get_unique_board_id_string(serial, sizeof(serial));
    for (len = 0; serial[len] && len < 32; len++)
      desc_str[1 + len] = (uint16_t)serial[len];
  } else {
    if (index >= TU_ARRAY_SIZE(string_desc_arr) || !string_desc_arr[index])
      return NULL;
    const char *s = string_desc_arr[index];
    for (len = 0; s[len] && len < 32; len++)
      desc_str[1 + len] = (uint16_t)s[len];
  }

  desc_str[0] = (uint16_t)((TUSB_DESC_STRING << 8) | (2 * len + 2));
  return desc_str;
}

// Opening CDC0 at 1200 baud reboots into the UF2 bootloader (Arduino
// convention). gw.exe never uses 1200 baud.
void tud_cdc_line_coding_cb(uint8_t itf, const cdc_line_coding_t *coding) {
  if (itf == 0 && coding->bit_rate == 1200)
    reset_usb_boot(0, 0);
}
