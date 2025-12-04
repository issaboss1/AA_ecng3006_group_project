// ESP8266 + ADS1115 SpO2 Measurement
// IR sensor on A0, RED sensor on A1
// Outputs SpO2 value

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
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
#define ADS_MUX_AIN0_GND    (4 << 12)   // IR on A0
#define ADS_MUX_AIN1_GND    (5 << 12)   // RED on A1
#define ADS_PGA_4_096V      (1 << 9)    // +/-4.096 V
#define ADS_MODE_SINGLESHOT (1 << 8)
#define ADS_DR_128SPS       (4 << 5)    // sample rate
#define ADS_COMP_DISABLE    (3 << 0)

static const char *TAG = "SPO2";


//  Write to ADS1115 register
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

    return ret == ESP_OK;
}


//  Read from ADS1115 register
static bool ads_read_reg(uint8_t reg, uint16_t *out) {
    uint8_t data[2];
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    // select register
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ADS1115_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);

    // read 2 bytes
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ADS1115_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, 2, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(I2C_PORT, cmd, 20 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK) return false;

    *out = ((uint16_t)data[0] << 8) | data[1];
    return true;
}


//  ESP8266 I2C initialization
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

//  Configure ADS1115 channel
static bool ads_set_channel(uint8_t channel) {
    uint16_t mux;

    if (channel == 0) mux = ADS_MUX_AIN0_GND; // IR
    else              mux = ADS_MUX_AIN1_GND; // RED

    uint16_t config =
        ADS_OS_SINGLE |
        mux |
        ADS_PGA_4_096V |
        ADS_MODE_SINGLESHOT |
        ADS_DR_128SPS |
        ADS_COMP_DISABLE;

    return ads_write_reg(ADS_REG_CONFIG, config);
}

//  Read voltage from ADS1115 channel
static bool ads_read_voltage(uint8_t channel, float *vout) {
    const float VREF = 4.096f;

    if (!ads_set_channel(channel)) return false;

    vTaskDelay(pdMS_TO_TICKS(10)); // conversion time

    uint16_t raw;
    if (!ads_read_reg(ADS_REG_CONVERT, &raw)) return false;

    int16_t value = (int16_t)raw;
    *vout = (value * VREF) / 32768.0f;
    return true;
}


//      SpO2 PROCESSING TASK
static void spo2_task(void *arg) {
    ESP_LOGI(TAG, "SpO2 task running...");

    float dc_ir = 0, dc_red = 0;
    float ac_ir = 0, ac_red = 0;

    float ac_ir_env = 0, ac_red_env = 0;

    const float alpha_dc = 0.01f;   // slow DC filter
    const float alpha_ac = 0.15f;   // faster AC envelope

    float spo2 = 0;

    while (1) {
        float ir, red;

        if (!ads_read_voltage(0, &ir)) continue;   // IR channel
        if (!ads_read_voltage(1, &red)) continue;  // RED channel

        // Initialize
        if (dc_ir == 0) dc_ir = ir;
        if (dc_red == 0) dc_red = red;

        // Update DC estimate
        dc_ir += alpha_dc * (ir - dc_ir);
        dc_red += alpha_dc * (red - dc_red);

        // AC signal (pulse)
        ac_ir = ir - dc_ir;
        ac_red = red - dc_red;

        // Envelope of AC (absolute)
        ac_ir_env  += alpha_ac * (fabs(ac_ir)  - ac_ir_env);
        ac_red_env += alpha_ac * (fabs(ac_red) - ac_red_env);

        // Ensure valid data
        if (dc_ir > 0.01 && dc_red > 0.01 && ac_ir_env > 0.0005 && ac_red_env > 0.0005) {

            float ratio_ir  = ac_ir_env / dc_ir;
            float ratio_red = ac_red_env / dc_red;

            float R = ratio_red / ratio_ir;

            // Simple linear estimate (tune A/B for accuracy)
            float A = 110.0f;
            float B = 25.0f;
            float spo2_new = A - B * R;

            if (spo2_new < 0) spo2_new = 0;
            if (spo2_new > 100) spo2_new = 100;

            // Smooth output
            spo2 = spo2 + 0.2f * (spo2_new - spo2);

            int spo2_int = (int)(spo2 + 0.5f);

            printf("IR=%.3f  RED=%.3f  SpO2=%d%%\n", ir, red, spo2_int);
        }

        vTaskDelay(pdMS_TO_TICKS(20)); // ~50 Hz display
    }
}

void app_main(void) {
    init_i2c();
    xTaskCreate(spo2_task, "spo2_task", 4096, NULL, 5, NULL);
}