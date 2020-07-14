#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "sdkconfig.h"

#include "freertos/event_groups.h"

#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_system.h"

#include "lwip/err.h"
#include "lwip/sys.h"

#include "utils.h"
#include "constants.h"
#include "wifi_manager/wifi_manager.h"

void monitoring_task(void *pvParameter)
{
	for(;;){
		ESP_LOGI(TAG_BASE, "free heap: %d", esp_get_free_heap_size());
		vTaskDelay(pdMS_TO_TICKS(10000));
	}
}

void cb_connection_established(void *pvParameter){
	ESP_LOGI(TAG_BASE, "I have a connection!");
}

void app_main(void)
{
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);

    ESP_LOGI(TAG_BASE, "MiHome ESP32 Base v0.0.1 (%s chip, %d CPU cores, WiFi%s%s, revision %d, %dMB %s flash, free heap: %d)",
            CONFIG_IDF_TARGET,
            chip_info.cores,
            (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
            (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "",
            chip_info.revision,
            spi_flash_get_chip_size() / (1024 * 1024),
            (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external",
            esp_get_free_heap_size());


    gpio_pad_select_gpio(BLINK_GPIO);
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);

		//wifi_scanner();

		wifi_manager_start();
  	wifi_manager_set_callback(EVENT_STA_GOT_IP, &cb_connection_established);
  	//xTaskCreatePinnedToCore(&monitoring_task, "monitoring_task", 2048, NULL, 1, NULL, 1);

    while(true){
        gpio_set_level(BLINK_GPIO, 0);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        gpio_set_level(BLINK_GPIO, 1);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

/*
idf.py build
idf.py -p /dev/cu.usbserial-D3070SV6 flash

IDF_PATH=/Users/johnosullivan/esp/esp-idf
IDF_TOOLS_EXPORT_CMD=/Users/johnosullivan/esp/esp-idf/export.sh
IDF_TOOLS_INSTALL_CMD=/Users/johnosullivan/esp/esp-idf/install.sh
IDF_PYTHON_ENV_PATH=/Users/johnosullivan/.espressif/python_env/idf4.2_py2.7_env
*/
