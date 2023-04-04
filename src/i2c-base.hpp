#include <driver/i2c.h>
#include <stdio.h>
#pragma once

#define I2C_ERR_CHECK(expr, msg) { \
  auto err = (expr); \
  if (err != ESP_OK) { \
    printf(msg, err); \
  } \
}
esp_err_t i2c_read8(uint8_t addr, uint8_t reg, uint8_t *data);
esp_err_t i2c_read(uint8_t addr, uint8_t reg, uint8_t *buffer, uint16_t readlen);
esp_err_t i2c_write8(uint8_t addr, uint8_t reg, uint8_t data);