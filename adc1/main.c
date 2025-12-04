// ADS1115 + ESP8266 ESP-01 test
// A0 = potentiometer center pin

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"

#define SDA_PIN 2
#define SCL_PIN 0
#define I2C_PORT 0
#define I2C_FREQ 100000

#define ADS1115_ADDR 0x48
#define REG_CONVERT 0x00
#define REG_CONFIG  0x01

static const char *TAG = "ADS1115";

static bool write_reg(uint8_t reg, uint16_t val) {
    uint8_t data[3] = { reg, val >> 8, val & 0xFF };

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ADS1115_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, data, 3, true);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(I2C_PORT, cmd, 20 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    return ret == ESP_OK;
}

static bool read_reg(uint8_t reg, uint16_t *out) {
    uint8_t buf[2];

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ADS1115_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ADS1115_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, buf, 2, I2C_MASTER_LAST_NACK);

    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_PORT, cmd, 20 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    *out = (buf[0] << 8) | buf[1];
    return ret == ESP_OK;
}

static void init_i2c(void) {
    i2c_config_t cfg;
    cfg.mode = I2C_MODE_MASTER;
    cfg.sda_io_num = SDA_PIN;
    cfg.sda_pullup_en = true;
    cfg.scl_io_num = SCL_PIN;
    cfg.scl_pullup_en = true;
    cfg.clk_stretch_tick = 300;

    i2c_driver_install(I2C_PORT, cfg.mode);
    i2c_param_config(I2C_PORT, &cfg);
}

static bool init_ads(void) {
    uint16_t config =
        (1 << 15) | 
        (4 << 12) |
        (1 << 9)  |
        (0 << 8)  |
        (4 << 5)  |
        (3 << 0);

    return write_reg(REG_CONFIG, config);
}

void ads_task(void *arg) {
    init_ads();
    ESP_LOGI(TAG, "Reading ADC...");

    while (1) {
        uint16_t cfg;
        read_reg(REG_CONFIG, &cfg);
        cfg |= (1 << 15);
        write_reg(REG_CONFIG, cfg);

        vTaskDelay(pdMS_TO_TICKS(10));

        uint16_t raw;
        if (read_reg(REG_CONVERT, &raw)) {
            ESP_LOGI(TAG, "RAW = %d", (int16_t)raw);
        }

        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

void app_main(void) {
    init_i2c();
    xTaskCreate(ads_task, "ads_task", 4096, NULL, 5, NULL);
}
