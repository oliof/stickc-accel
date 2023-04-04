#include "i2c-base.hpp"

esp_err_t i2c_read8(uint8_t addr, uint8_t reg, uint8_t *data) {
    auto cmd = i2c_cmd_link_create();
    ESP_ERROR_CHECK(i2c_master_start(cmd));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, I2C_MASTER_ACK));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, reg, I2C_MASTER_ACK));
    ESP_ERROR_CHECK(i2c_master_start(cmd));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_READ, I2C_MASTER_ACK));
    ESP_ERROR_CHECK(i2c_master_read_byte(cmd, data, I2C_MASTER_NACK));
    ESP_ERROR_CHECK(i2c_master_stop(cmd));
    esp_err_t err = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    return err;
}

esp_err_t i2c_read(uint8_t addr, uint8_t reg, uint8_t *buffer, uint16_t readlen) {
    auto cmd = i2c_cmd_link_create();
    ESP_ERROR_CHECK(i2c_master_start(cmd));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, I2C_MASTER_ACK));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, reg, I2C_MASTER_ACK));
    ESP_ERROR_CHECK(i2c_master_start(cmd));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_READ, I2C_MASTER_ACK));
    ESP_ERROR_CHECK(i2c_master_read(cmd, buffer, readlen, I2C_MASTER_LAST_NACK));
    ESP_ERROR_CHECK(i2c_master_stop(cmd));
    esp_err_t err = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    return err;
}

esp_err_t i2c_write8(uint8_t addr, uint8_t reg, uint8_t data) {
    auto cmd = i2c_cmd_link_create();
    ESP_ERROR_CHECK(i2c_master_start(cmd));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, I2C_MASTER_ACK));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, reg, I2C_MASTER_ACK));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, data, I2C_MASTER_ACK));
    ESP_ERROR_CHECK(i2c_master_stop(cmd));
    esp_err_t err = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    return err;
}