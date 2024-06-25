#ifndef __bq27441_
#define __bq27441_

#include "esp_system.h"

esp_err_t bq27441_init(void);
esp_err_t bq27441_update_config(void);
esp_err_t bq27441_get_soc(uint16_t *soc);
esp_err_t bq27441_get_voltage(uint16_t *voltage);
esp_err_t bq27441_get_temperature(uint16_t *temperature);
esp_err_t bq27441_get_ave_current(uint16_t *ave_current);
esp_err_t bq27441_hard_reset();
esp_err_t bq27441_shutdown(void);

esp_err_t bq27441_control_read(uint16_t function, uint16_t* value);
esp_err_t bq27441_control_write(uint16_t function);
esp_err_t bq27441_seal(void);
esp_err_t bq27441_unseal(void);
esp_err_t bq27441_enter_config_update(void);
esp_err_t bq27441_leave_config_update(void);

esp_err_t bq27441_read_byte(uint8_t reg, uint8_t *value);
esp_err_t bq27441_read_word(uint8_t reg, uint16_t *value);
esp_err_t bq27441_write_word(uint8_t reg, uint16_t value);

#endif // __bq27441_