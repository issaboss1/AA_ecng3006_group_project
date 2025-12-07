// ADS1115 + ESP8266 ESP-01
// A0 = PPG amplifier output (or your amplifier output)

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"

#define SDA_PIN         2
#define SCL_PIN         0
#define I2C_PORT        0

#define ADS1115_ADDR    0x48
#define ADS_REG_CONVERT 0x00
#define ADS_REG_CONFIG  0x01

// ADS1115 config bits
#define ADS_OS_SINGLE       (1 << 15)
#define ADS_MUX_AIN0_GND    (4 << 12)
#define ADS_PGA_4_096V      (1 << 9)    // +/-4.096 V
#define ADS_MODE_SINGLESHOT (1 << 8)
#define ADS_DR_128SPS       (4 << 5)
#define ADS_COMP_DISABLE    (3 << 0)

static const char *TAG = "ADS1115";

// write 16-bit value to ADS1115 register
static bool ads_write_reg(uint8_t reg, uint16_t value) {
    uint8_t data[3];
    data[0] = reg;
    data[1] = (value >> 8) & 0xFF;
    data[2] = value & 0xFF;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ADS1115_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, data, 3, true);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(I2C_PORT, cmd, 20 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    return (ret == ESP_OK);
}

// read 16-bit value from ADS1115 register
static bool ads_read_reg(uint8_t reg, uint16_t *out_value) {
    uint8_t buf[2];

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    // select register
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ADS1115_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);

    // read two bytes
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ADS1115_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, buf, 2, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(I2C_PORT, cmd, 20 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "i2c read failed, err=%d", ret);
        return false;
    }

    *out_value = ((uint16_t)buf[0] << 8) | buf[1];
    return true;
}

// I2C init for ESP8266_RTOS_SDK
static void init_i2c(void) {
    i2c_config_t cfg;
    cfg.mode = I2C_MODE_MASTER;
    cfg.sda_io_num = SDA_PIN;
    cfg.sda_pullup_en = true;
    cfg.scl_io_num = SCL_PIN;
    cfg.scl_pullup_en = true;
    cfg.clk_stretch_tick = 300;   // default

    // ESP8266 API
    i2c_driver_install(I2C_PORT, cfg.mode);
    i2c_param_config(I2C_PORT, &cfg);
}

// ADS1115 init, single-shot on AIN0
static bool init_ads(void) {
    uint16_t config =
        ADS_OS_SINGLE       |   // start single conversion
        ADS_MUX_AIN0_GND    |   // AIN0 vs GND
        ADS_PGA_4_096V      |   // full-scale range
        ADS_MODE_SINGLESHOT |   // single-shot mode
        ADS_DR_128SPS       |   // 128 samples per second
        ADS_COMP_DISABLE;       // comparator off

    return ads_write_reg(ADS_REG_CONFIG, config);
}

// task: read ADS1115 and estimate BPM from PPG peaks
static void ads_task(void *arg) {
    if (!init_ads()) {
        ESP_LOGE(TAG, "ADS1115 init failed");
    } else {
        ESP_LOGI(TAG, "ADS1115 init successful");
    }

    const float VREF = 4.096f;            // matches ADS_PGA_4_096V
    const float THRESHOLD_V = 0.3f;       // start low, tune later
    const uint32_t MIN_INTERVAL_MS = 350; // min time between beats
    const float BPM_ALPHA = 0.3f;         // smoothing factor

    float prev_voltage = 0.0f;
    uint32_t last_peak_time_ms = 0;
    float bpm = 0.0f;

    while (1) {
        // start a single conversion by setting OS bit
        uint16_t cfg;
        if (!ads_read_reg(ADS_REG_CONFIG, &cfg)) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        cfg |= ADS_OS_SINGLE;
        ads_write_reg(ADS_REG_CONFIG, cfg);

        // wait for conversion
        vTaskDelay(pdMS_TO_TICKS(10));   // about 128 samples per second

        // read conversion
        uint16_t raw;
        if (!ads_read_reg(ADS_REG_CONVERT, &raw)) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        // convert to signed and voltage
        int16_t raw_signed = (int16_t)raw;
        float voltage = ((float)raw_signed * VREF) / 32768.0f;

        // print waveform so you see what is happening
        // use Arduino serial plotter or similar to see pulses
        printf("V=%.4f\n", voltage);

        // current time in ms
        TickType_t now_ticks = xTaskGetTickCount();
        uint32_t now_ms = now_ticks * portTICK_PERIOD_MS;

        // rising crossing of threshold
        if (voltage > THRESHOLD_V && prev_voltage <= THRESHOLD_V) {
            uint32_t interval_ms = now_ms - last_peak_time_ms;

            if (last_peak_time_ms != 0 && interval_ms > MIN_INTERVAL_MS) {
                float inst_bpm = 60000.0f / (float)interval_ms;

                if (bpm == 0.0f) {
                    bpm = inst_bpm;
                } else {
                    bpm = bpm + BPM_ALPHA * (inst_bpm - bpm);
                }

                printf("BPM=%d\n", (int)bpm);
                vTaskDelay(300);
            }

            last_peak_time_ms = now_ms;
        }

        prev_voltage = voltage;
    }
}

void app_main(void) {
    init_i2c();
    xTaskCreate(ads_task, "ads_task", 4096, NULL, 5, NULL);
}
