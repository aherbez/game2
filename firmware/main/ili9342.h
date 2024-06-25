#ifndef __ili9342_
#define __ili9342_

#include "esp_system.h"
#include "driver/spi_master.h"

esp_err_t ili9342_init(void);

esp_err_t ili9342_get_id(spi_device_handle_t spi, uint32_t* id);
esp_err_t ili9342_cmd(spi_device_handle_t spi, const uint8_t cmd);

#endif // __ili9342_