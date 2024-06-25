/* i2c-tools example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "bq27441.h"
#include "ili9342.h"

static const char *TAG = "microgame2";

void app_main(void)
{
    ESP_LOGI(TAG, "Hello world");

    if( bq27441_init() != ESP_OK ) {
        ESP_LOGI(TAG, "Failed to init bq27441.");
        return;
    }

    if( bq27441_update_config() != ESP_OK ) {
        ESP_LOGI(TAG, "Failed to update bq27441 config.");
        return;
    }

    if( ili9342_init() != ESP_OK ) {
        ESP_LOGI(TAG, "Failed to init ili9342.");
        return;
    }

    while(1) {
        uint8_t flags1, flags2;
        bq27441_read_byte(0x06, &flags1);
        bq27441_read_byte(0x07, &flags2);
        ESP_LOGI(TAG, "flags 0x%x 0x%x", flags1, flags2);

        // attempt to read the device type
        uint16_t control = 0x0;
        if( bq27441_write_word(0x0, 0x0) != ESP_OK ) {
            ESP_LOGI(TAG, "Failed to query bq27441 for status.");
            return;
        } 
        if( bq27441_read_word(0x0, &control) != ESP_OK ) {
            ESP_LOGI(TAG, "Failed to query bq27441 for status.");
            return;
        } 
        ESP_LOGI(TAG, "control 0x%x", control);

        // attempt to read the device type
        uint16_t device_type = 0x0;
        if( bq27441_write_word(0x0, 0x1) != ESP_OK ) {
            ESP_LOGI(TAG, "Failed to query bq27441 for status.");
            return;
        } 
        if( bq27441_read_word(0x0, &device_type) != ESP_OK ) {
            ESP_LOGI(TAG, "Failed to query bq27441 for device type.");
            return;
        } 
        ESP_LOGI(TAG, "device type 0x%x", device_type);


        uint16_t soc;
        if( bq27441_get_soc(&soc) != ESP_OK ) {
            ESP_LOGI(TAG, "Failed to query bq27441 for soc.");
            return;
        }    
        ESP_LOGI(TAG, "Found bq27441 soc %d.", soc);

        uint16_t voltage;
        if( bq27441_get_voltage(&voltage) != ESP_OK ) {
            ESP_LOGI(TAG, "Failed to query bq27441 for voltage.");
            return;
        }    
        ESP_LOGI(TAG, "Found bq27441 voltage %d.", voltage);

        uint16_t ave_current;
        if( bq27441_get_ave_current(&ave_current) != ESP_OK ) {
            ESP_LOGI(TAG, "Failed to query bq27441 for ave_current.");
            return;
        }    
        ESP_LOGI(TAG, "Found bq27441 average current %d.", ave_current);

        uint16_t temperature;
        if( bq27441_get_temperature(&temperature) != ESP_OK ) {
            ESP_LOGI(TAG, "Failed to query bq27441 for temperature.");
            return;
        }    
        ESP_LOGI(TAG, "Found bq27441 temperature %d.", temperature);

        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
}
