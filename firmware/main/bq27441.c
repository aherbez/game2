#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "driver/i2c.h"
#include "bq27441.h"


#define I2C_MASTER_TX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define ACK_CHECK_EN 0x1            /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0           /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                 /*!< I2C ack value */
#define NACK_VAL 0x1                /*!< I2C nack value */

static gpio_num_t i2c_gpio_sda = 12;
static gpio_num_t i2c_gpio_scl = 14;
static uint32_t i2c_frequency = 100000;
static i2c_port_t i2c_port = I2C_NUM_0;
static int chip_addr = 0x55;

static const char *TAG = "bq27441";

esp_err_t bq27441_read_byte(uint8_t reg, uint8_t *value) {
    esp_err_t result;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, chip_addr << 1 | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, chip_addr << 1 | I2C_MASTER_READ, ACK_CHECK_EN);
    i2c_master_read_byte(cmd, value, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    result = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
    if( result != ESP_OK ) {
        i2c_cmd_link_delete(cmd);
        return result;
    }
    i2c_cmd_link_delete(cmd);
    return ESP_OK;
}

esp_err_t bq27441_read_data(uint8_t reg, uint8_t *data, uint32_t len) {
    esp_err_t result;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, chip_addr << 1 | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, chip_addr << 1 | I2C_MASTER_READ, ACK_CHECK_EN);
    i2c_master_read(cmd, data, len, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    result = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
    if( result != ESP_OK ) {
        i2c_cmd_link_delete(cmd);
        return result;
    }
    i2c_cmd_link_delete(cmd);
    return ESP_OK;
}

esp_err_t bq27441_read_word(uint8_t reg, uint16_t *value) {
    return bq27441_read_data(reg, (uint8_t *)value, 2);
}

esp_err_t bq27441_read_flipped_word(uint8_t reg, uint16_t *value) {
    esp_err_t result = bq27441_read_data(reg, (uint8_t *)value, 2);
    if( result != ESP_OK ) {
        return result;
    }
    *value = ((*value) >> 8) | ((*value) << 8);
    return ESP_OK;
}

esp_err_t bq27441_write_byte(uint8_t reg, uint8_t value) {
    esp_err_t result;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, chip_addr << 1 | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, value, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    result = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
    if( result != ESP_OK ) {
        i2c_cmd_link_delete(cmd);
        return result;
    }
    i2c_cmd_link_delete(cmd);
    return ESP_OK;
}

esp_err_t bq27441_write_word(uint8_t reg, uint16_t value) {
    esp_err_t result;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, chip_addr << 1, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
    i2c_master_write(cmd, (uint8_t *)&value, 2, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    result = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
    if( result != ESP_OK ) {
        i2c_cmd_link_delete(cmd);
        return result;
    }
    i2c_cmd_link_delete(cmd);
    return ESP_OK;
}

esp_err_t bq27441_write_flipped_word(uint8_t reg, uint16_t value) {
    uint16_t flipped_value = ((value) >> 8) | ((value) << 8);
    return bq27441_write_word(reg, flipped_value);
}

esp_err_t bq27441_control_read(uint16_t function, uint16_t* value) {
    esp_err_t result;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, chip_addr << 1 | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, 0, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, 0, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, function & 0xff, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, function >> 8, ACK_CHECK_EN);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, chip_addr << 1 | I2C_MASTER_READ, ACK_CHECK_EN);
    i2c_master_read(cmd, (uint8_t*)value, 2, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    result = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
    if( result != ESP_OK ) {
        ESP_LOGI(TAG, "failed to issue command because %s", esp_err_to_name(result));
        i2c_cmd_link_delete(cmd);
        return result;
    }
    i2c_cmd_link_delete(cmd);
    return ESP_OK;
}

esp_err_t bq27441_control_write(uint16_t function) {
    esp_err_t result;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, chip_addr << 1 | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, 0, ACK_CHECK_EN);
    i2c_master_write(cmd, (uint8_t*)&function, 2, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    result = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
    if( result != ESP_OK ) {
        ESP_LOGI(TAG, "failed to issue command because %s", esp_err_to_name(result));
        i2c_cmd_link_delete(cmd);
        return result;
    }
    i2c_cmd_link_delete(cmd);
    return ESP_OK;
}

esp_err_t bq27441_unseal(void) {
    esp_err_t result = bq27441_write_word(0x00, 0x8000);
    if( result != ESP_OK ) {
        return result;
    }
    result = bq27441_write_word(0x00, 0x8000);
    if( result != ESP_OK ) {
        return result;
    }
    return ESP_OK;
}

esp_err_t bq27441_seal(void) {
    esp_err_t result = bq27441_control_write(0x20);
    if( result != ESP_OK ) {
        return result;
    }
    return ESP_OK;
}

esp_err_t bq27441_enter_config_update(void) {
    esp_err_t result = bq27441_write_word(0x00, 0x0013);
    if( result != ESP_OK ) {
        return result;
    }
    ESP_LOGI(TAG, "wrote enter config");
    uint8_t value = 0;
    while(!(value & (1 << 4))) {
        vTaskDelay(50 / portTICK_PERIOD_MS);
        result = bq27441_read_byte(0x06, &value);
        if( result != ESP_OK ) {
            ESP_LOGI(TAG, "failed to read byte because %s", esp_err_to_name(result));
            return result;
        }
    }
    return ESP_OK;
}

esp_err_t bq27441_leave_config_update(void) {
    esp_err_t result = bq27441_write_word(0x00, 0x0042);
    if( result != ESP_OK ) {
        return result;
    }
    uint8_t value = 0xff;
    while(value & (1 << 4)) {
        result = bq27441_read_byte(0x06, &value);
        if( result != ESP_OK ) {
            return result;
        }
    }
    return ESP_OK;
}

esp_err_t bq27441_update_config(void) {
    esp_err_t result = bq27441_unseal();
    if( result != ESP_OK ) {
        return result;
    }
    ESP_LOGI(TAG, "Unseal ok");
    result = bq27441_enter_config_update();
    if( result != ESP_OK ) {
        return result;
    }
    ESP_LOGI(TAG, "Enter config ok");
    // setup block ram transfer
    result = bq27441_write_byte(0x61, 0x00);
    if( result != ESP_OK ) {
        return result;
    }
    result = bq27441_write_byte(0x3e, 0x52);
    if( result != ESP_OK ) {
        return result;
    }
    result = bq27441_write_byte(0x3f, 0x00);
    if( result != ESP_OK ) {
        return result;
    }    uint8_t updated_checksum = 0, new_checksum = 0;    
    do 
    {
        uint16_t new_dc = 2000, new_de = 2000 * 3.7, new_tv = 4000, new_tr = 2000 / (0.1 * 46);
        result = bq27441_write_flipped_word(0x4a, new_dc);
        if( result != ESP_OK ) {
            return result;
        }
        result = bq27441_write_flipped_word(0x4c, new_de);
        if( result != ESP_OK ) {
            return result;
        }
        result = bq27441_write_flipped_word(0x50, new_tv);
        if( result != ESP_OK ) {
            return result;
        }
        result = bq27441_write_flipped_word(0x5b, new_tr);
        if( result != ESP_OK ) {
            return result;
        }
        uint8_t data2[32];
        result = bq27441_read_data(0x40, data2, 32);
        if( result != ESP_OK ) {
            return result;
        }
        uint32_t temp_checksum = 0;
        for( uint32_t i = 0; i < 32; i++ ) {
            temp_checksum = (temp_checksum + data2[i]) & 0xff;
        }
        new_checksum = 0xff - (temp_checksum & 0xff);
        result = bq27441_write_byte(0x60, new_checksum);
        if( result != ESP_OK ) {
            return result;
        }
        result = bq27441_write_byte(0x3e, 0x52);
        if( result != ESP_OK ) {
            return result;
        }
        result = bq27441_write_byte(0x3f, 0x00);
        if( result != ESP_OK ) {
            return result;
        }
        result = bq27441_read_byte(0x60, &updated_checksum);
        if( result != ESP_OK ) {
            return result;
        }
    } while(updated_checksum != new_checksum);
    result = bq27441_write_word(0x00, 0x42);
    if( result != ESP_OK ) {
        return result;
    }
    result = bq27441_leave_config_update();
    if( result != ESP_OK ) {
        return result;
    }
    result = bq27441_write_word(0x00, 0x20);
    if( result != ESP_OK ) {
        return result;
    }
    return ESP_OK;    
}

esp_err_t bq27441_init(void) {
    esp_err_t result;
    // install the i2c driver
    result = i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
    if( result != ESP_OK ) {
        return result;
    }
    ESP_LOGI(TAG, "i2c driver installed ok");
    // configure the i2c driver
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = i2c_gpio_sda,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = i2c_gpio_scl,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = i2c_frequency
    };
    result = i2c_param_config(i2c_port, &conf);
    if( result != ESP_OK) {
        return result;
    }
    i2c_set_timeout(I2C_NUM_0, 1048575);
    ESP_LOGI(TAG, "i2c driver param configured ok");
/*
    // attempt to read the device type
    uint16_t device_type = 0x0;
    result = bq27441_control_read(0x1, &device_type);
    if( result != ESP_OK ) {
        return result;
    } 
    ESP_LOGI(TAG, "heard 0x%x from bq27441.", device_type);
    // TODO: why do we not see the expected value here?
    if( device_type != 0x0421 ) {
        //return ESP_ERR_NOT_SUPPORTED;
    } */
    return ESP_OK;
}

esp_err_t bq27441_shutdown(void) {
    esp_err_t result;
    // release the driver
    result = i2c_driver_delete(I2C_NUM_0);
    if( result != ESP_OK ) {
        return result;
    }
    return ESP_OK;
}

esp_err_t bq27441_get_soc(uint16_t *soc) {   
    esp_err_t result;
    result = bq27441_read_word(0x1c, soc);
    if( result != ESP_OK ) {
        return result;
    }
    return ESP_OK;    
}

esp_err_t bq27441_get_voltage(uint16_t *voltage) {   
    esp_err_t result = bq27441_read_word(0x4, voltage);
    if( result != ESP_OK ) {
        return result;
    }
    return ESP_OK;    
}

esp_err_t bq27441_get_temperature(uint16_t *temperature) {   
    esp_err_t result = bq27441_read_word(0x2, temperature);
    if( result != ESP_OK ) {
        return result;
    }
    // confirm from 1/10th of a Kelvin to Fahrenheit
    *temperature = ((*temperature) * 9 / 5 - 4597) / 10;
    return ESP_OK;    
}

esp_err_t bq27441_get_ave_current(uint16_t *ave_current) {   
    esp_err_t result = bq27441_read_word(0x10, ave_current);
    if( result != ESP_OK ) {
        return result;
    }
    return ESP_OK;    
}

esp_err_t bq27441_hard_reset() {   
    esp_err_t result = bq27441_unseal();
    if( result != ESP_OK ) {
        return result;
    }
    result = bq27441_write_word(0x00, 0x41);
    if( result != ESP_OK ) {
        return result;
    }
    return ESP_OK;    
}
