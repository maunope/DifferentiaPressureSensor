////////////////////////Mass storage over USB////////////////////////
// Define these at the top of a header file (e.g., usb_config.h)
// Or directly in your main.c file before any TinyUSB includes

#define CONFIG_TINYUSB_ENABLED 1
#define CONFIG_TINYUSB_MSC_ENABLED 1
#define CONFIG_TINYUSB_MSC_SFLASH_ENABLED 0
#define CONFIG_TINYUSB_MSC_SDCARD_ENABLED 1
#define CONFIG_TINYUSB_CDC_ENABLED 0
#define CONFIG_TINYUSB_DFU_ENABLED 0
#define CONFIG_TINYUSB_HID_ENABLED 0
#define CONFIG_TINYUSB_MIDI_ENABLED 0

// You may also need to define the MCU and OS
#define CFG_TUSB_MCU OPT_MCU_ESP32S3
#define CFG_TUSB_OS OPT_OS_ESP_IDF

////////////////////////Mass storage over USB////////////////////////