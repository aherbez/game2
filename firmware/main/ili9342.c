#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "ili9342.h"

static const char *TAG = "ili9342";

#define DMA_CHAN    2
#define PIN_NUM_DCX 2
#define PIN_NUM_RST 4
#define PIN_NUM_SDA 5
#define PIN_NUM_CSX 13
#define PIN_NUM_SCL 15

// To speed up transfers, every SPI transfer sends a bunch of lines. This define specifies how many. More means more memory use,
//  but less overhead for setting up / finishing transfers. Make sure 240 is dividable by this.
#define PARALLEL_LINES 16

// This function is called (in irq context!) just before a transmission starts. It will
//  set the D/CX line to the value indicated in the user field.
void ili9342_spi_pre_transfer_callback(spi_transaction_t *t)
{
    int dc = (int)t->user;
    gpio_set_level(PIN_NUM_DCX, dc);
}

esp_err_t ili9342_init(void) {
    esp_err_t result;
    spi_device_handle_t spi;
    spi_bus_config_t buscfg={
        .miso_io_num=-1,
        .mosi_io_num=PIN_NUM_SDA,
        .sclk_io_num=PIN_NUM_SCL,
        .quadwp_io_num=-1,
        .quadhd_io_num=-1,
        .max_transfer_sz=PARALLEL_LINES*320*2+8
    };
    spi_device_interface_config_t devcfg={
#ifdef CONFIG_LCD_OVERCLOCK
        .clock_speed_hz=26*1000*1000,           //Clock out at 26 MHz
#else
        .clock_speed_hz=10*1000*1000,           //Clock out at 10 MHz
#endif
        .mode=0,                                //SPI mode 0
        .spics_io_num=PIN_NUM_CSX,              //CS pin
        .queue_size=7,                          //We want to be able to queue 7 transactions at a time
        .pre_cb=ili9342_spi_pre_transfer_callback,  //Specify pre-transfer callback to handle D/C line
        .flags=SPI_DEVICE_HALFDUPLEX|SPI_DEVICE_3WIRE
    };
    // Initialize the SPI bus
    result = spi_bus_initialize(HSPI_HOST, &buscfg, DMA_CHAN);
    if( result != ESP_OK ) {
        return result;
    }
    // Attach the LCD to the SPI bus
    result = spi_bus_add_device(HSPI_HOST, &devcfg, &spi);
    if( result != ESP_OK ) {
        return result;
    }

    // Configure the gpio lines
    gpio_set_direction(PIN_NUM_DCX, GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_NUM_RST, GPIO_MODE_OUTPUT);

    ESP_LOGI(TAG, "resetting LCD");

    // Reset the display
    gpio_set_level(PIN_NUM_RST, 0);
    vTaskDelay(100 / portTICK_RATE_MS);
    gpio_set_level(PIN_NUM_RST, 1);
    vTaskDelay(100 / portTICK_RATE_MS);

    ESP_LOGI(TAG, "LCD reset");

    uint32_t id;
    result = ili9342_get_id(spi, &id);
    if( result != ESP_OK ) {
        return result;
    }

    ESP_LOGI(TAG, "LCD id 0x%x", id);

    return ESP_OK;
}

esp_err_t ili9342_cmd(spi_device_handle_t spi, const uint8_t cmd) {
    esp_err_t result;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));                           // Zero out the transaction
    t.length = 8;                                       // Command is 8 bits
    t.tx_buffer = &cmd;                                 // The data is the cmd itself
    t.user = (void*)0;                                  // D/C needs to be set to 0
    result = spi_device_polling_transmit(spi, &t);      // Transmit!
    if( result != ESP_OK ) {
        return result;
    }
    return ESP_OK;
}

esp_err_t ili9342_get_id(spi_device_handle_t spi, uint32_t* id) {
    esp_err_t result;
    // get_id cmd
    result = ili9342_cmd(spi, 0x04);
    if( result != ESP_OK ) {
        return result;
    }
    
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));                           // Zero out the transaction
    t.length= 8 * 3;                                    // Result is 24 bits
    //t.flags = SPI_TRANS_USE_RXDATA;
    t.rx_buffer = id;
    t.rxlength = 8 * 4;
    t.user = (void *)1;                                 // D/C needs to be set to 1
    result = spi_device_polling_transmit(spi, &t);      // Transmit!
    if( result != ESP_OK ) {
        return result;
    }
    *id = *(uint32_t *)t.rx_data;
    return ESP_OK;
}
