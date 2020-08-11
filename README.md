# Main Example

Starts a FreeRTOS task to blink an LED

See the README.md file in the upper level 'examples' directory for more information about examples.

wifi_scanner();

export IDF_PATH=~/esp/esp-idf
export PATH="$IDF_PATH/tools:$PATH"


idf.py build
idf.py -p /dev/cu.usbserial-D3070SV6 flash

IDF_PATH=/Users/johnosullivan/esp/esp-idf
IDF_TOOLS_EXPORT_CMD=/Users/johnosullivan/esp/esp-idf/export.sh
IDF_TOOLS_INSTALL_CMD=/Users/johnosullivan/esp/esp-idf/install.sh
IDF_PYTHON_ENV_PATH=/Users/johnosullivan/.espressif/python_env/idf4.2_py2.7_env
