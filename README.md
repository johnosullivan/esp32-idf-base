# Main Example

Starts a FreeRTOS task to blink an LED

See the README.md file in the upper level 'examples' directory for more information about examples.

wifi_scanner();

export IDF_PATH=~/esp/esp-idf
export PATH="$IDF_PATH/tools:$PATH"

idf.py build
idf.py -p /dev/cu.usbserial-1410 flash

IDF_PATH=/Users/johnosullivan/esp/esp-idf
IDF_TOOLS_EXPORT_CMD=/Users/johnosullivan/esp/esp-idf/export.sh
IDF_TOOLS_INSTALL_CMD=/Users/johnosullivan/esp/esp-idf/install.sh
IDF_PYTHON_ENV_PATH=/Users/johnosullivan/.espressif/python_env/idf4.2_py2.7_env

/usr/local/Cellar/gcc-arm-none-eabi/20180627/bin/arm-none-eabi-gcc

ln -s ~/nordic_sdk/nRF-CLT_10_9_0/nrfjprog/nrfjprog /usr/local/bin/nrfjprog
ln -s ~/nordic_sdk/nRF-CLT_10_9_0/mergehex/mergehex /usr/local/bin/mergehex

~/nordic_sdk/nRF5_SDK_17.0.0/components/toolchain/gcc/Makefile.posix

GNU_INSTALL_ROOT := /usr/local/Cellar/gcc-arm-none-eabi/20180627/bin/
GNU_VERSION := 7.3.1
GNU_PREFIX := arm-none-eabi

ls /dev/cu.*

nrfutil dfu --help

nrfutil pkg generate --application _build/nrf52832_xxaa.hex zip.zip --hw-version 52 --sd-req


nrfutil pkg generate --application _build/nrf52840_xxaa.bin test1.zip --hw-version 52 --sd-req 0xB6 --application-version 1

nrfutil --verbose dfu serial -pkg test1.zip -p /dev/cu.usbmodem14201 -b 115200

0x0052 --application firmware.hex dfu-package.zip

nrfutil dfu usb-serial -pkg test.zip -p /dev/cu.usbmodem14201 -b 115200

adafruit-nrfutil dfu usb-serial -pkg test.zip -p /dev/cu.usbmodem14201 -b 115200

nrfutil pkg generate --application _build/nrf52840_xxaa.bin test.zip --hw-version 52 --sd-req 0xCA --application-version 1

adafruit-nrfutil --help

nrfutil pkg display tzip.zip

adafruit-nrfutil --verbose dfu serial --package tzip.zip -p /dev/cu.usbmodem14201 -b 115200

adafruit-nrfutil dfu genpkg --dev-type 0x0052 --application firmware.hex dfu-package.zip

adafruit-nrfutil dfu serial --package /private/var/folders/xd/vh_d6szn6qq683970lq0xbxr0000gn/T/arduino_build_149399/blenordic.ino.zip -p /dev/cu.usbmodem14201 -b 115200

adafruit-nrfutil pkg generate --application _build/nrf52840_xxaa.hex test1.zip --hw-version 0x0052 --sd-req 0xCA --application-version 1


nrfutil pkg generate --hw-version 52 --sd-req 0x00 --application-version 1 --application _build/nrf52840_xxaa.bin  app_dfu_package.zip

#
# BT config
#
CONFIG_BT_ENABLED=y
CONFIG_BTDM_CTRL_MODE_BLE_ONLY=y
CONFIG_BTDM_CTRL_MODE_BR_EDR_ONLY=n
CONFIG_BTDM_CTRL_MODE_BTDM=n
CONFIG_BTDM_CTRL_PINNED_TO_CORE_0=y
CONFIG_BTDM_CTRL_PINNED_TO_CORE_1=n
CONFIG_BTDM_CTRL_PINNED_TO_CORE=0
CONFIG_BTDM_CTRL_HCI_MODE_VHCI=y
CONFIG_BTDM_CTRL_HCI_MODE_UART_H4=n
CONFIG_BT_BLUEDROID_ENABLED=y
CONFIG_BT_BLUEDROID_PINNED_TO_CORE_0=y
CONFIG_BT_BLUEDROID_PINNED_TO_CORE_1=n
CONFIG_BT_BLUEDROID_PINNED_TO_CORE=0
CONFIG_BT_BTC_TASK_STACK_SIZE=3072
CONFIG_BT_BLUEDROID_MEM_DEBUG=n
CONFIG_BT_CLASSIC_ENABLED=n
CONFIG_BT_GATTS_ENABLE=y
CONFIG_BT_GATTC_ENABLE=n
CONFIG_BT_BLE_SMP_ENABLE=n
CONFIG_BL_ENABLE_SRVCHG_REG=y
CONFIG_BT_STACK_NO_LOG=n
CONFIG_BT_ACL_CONNECTIONS=4
CONFIG_BT_ALLOCATION_FROM_SPIRAM_FIRST=n
CONFIG_BT_BLE_DYNAMIC_ENV_MEMORY=n
CONFIG_BT_SMP_ENABLE=n
