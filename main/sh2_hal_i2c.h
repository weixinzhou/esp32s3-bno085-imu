#pragma once
#include "sh2_hal.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"

// Build an sh2_Hal_t backed by the ESP-IDF I2C master driver plus the
// BNO085's NRST and H_INTN pins. The returned pointer stays valid for the
// lifetime of the program; pass it straight to sh2_open().
sh2_Hal_t *sh2_hal_i2c_create(gpio_num_t sda, gpio_num_t scl,
                              gpio_num_t intn, gpio_num_t nrst,
                              uint8_t i2c_addr, uint32_t scl_hz);

// The I2C bus the HAL opened, for other devices to share; NULL before open.
i2c_master_bus_handle_t sh2_hal_i2c_bus(void);
