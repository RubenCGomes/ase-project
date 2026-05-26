#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "mpu6050.h"
#include "st7735.h"

static const char *TAG = "MAIN";

// I2C Pin Configuration
#define I2C_SDA_PIN     6
#define I2C_SCL_PIN     7

// TFT SPI Pin Configuration (conforme o hardware)
#define PIN_MOSI        19
#define PIN_CLK         21
#define PIN_CS          22
#define PIN_DC          2
#define PIN_RST         3
#define PIN_BL          15

void app_main(void)
{
    i2c_master_bus_handle_t busHandle = NULL;
    i2c_master_dev_handle_t sensorHandle = NULL;

    ESP_LOGI(TAG, "Initializing TFT display...");
    st7735_config_t tftCfg = {
       .mosi_io_num = PIN_MOSI,
       .sclk_io_num = PIN_CLK,
       .cs_io_num = PIN_CS,
       .dc_io_num = PIN_DC,
       .rst_io_num = PIN_RST,
       .bl_io_num = PIN_BL,
       .host_id = SPI2_HOST
    };

    bool display_ok = false;
    if (st7735_init(&tftCfg) == ESP_OK) {
        display_ok = true;
        ESP_LOGI(TAG, "TFT display initialized successfully.");
        st7735_fill_screen(ST7735_BLACK);
        
        // Draw static headers
        st7735_draw_string(10, 8,  "MPU-6050 ACCEL", ST7735_CYAN, ST7735_BLACK, 1);
        st7735_draw_string(10, 18, "==============", ST7735_GRAY, ST7735_BLACK, 1);
    } else {
        ESP_LOGE(TAG, "Failed to initialize TFT display!");
    }

    ESP_LOGI(TAG, "Initializing MPU-6050 sensor on SDA: GPIO %d, SCL: GPIO %d", I2C_SDA_PIN, I2C_SCL_PIN);
    esp_err_t err = mpu6050_init(&busHandle, &sensorHandle, MPU6050_SENSOR_ADDR, 
                                 I2C_SDA_PIN, I2C_SCL_PIN, MPU6050_SCL_DFLT_FREQ_HZ);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize MPU-6050 accelerometer: %s", esp_err_to_name(err));
        if (display_ok) {
            st7735_draw_string(10, 36, "Sensor Init Err", ST7735_RED, ST7735_BLACK, 1);
        }
        return;
    }

    ESP_LOGI(TAG, "Entering sensor reading and display loop...");

    int16_t ax = 0, ay = 0, az = 0;
    char textBuf[32];
    
    while (1) {
        err = mpu6050_read_accel(sensorHandle, &ax, &ay, &az);
        if (err == ESP_OK) {
            // Default range is +/- 2g, sensitivity factor is 16384 LSB/g
            double ax_g = (double)ax / 16384.0;
            double ay_g = (double)ay / 16384.0;
            double az_g = (double)az / 16384.0;

            ESP_LOGI(TAG, "Accel Raw: X=%6d | Y=%6d | Z=%6d  ==>  Accel G: X=%6.3f | Y=%6.3f | Z=%6.3f g",
                     ax, ay, az, ax_g, ay_g, az_g);

            if (display_ok) {
                // Format strings with fixed padding to avoid character remnants
                snprintf(textBuf, sizeof(textBuf), "X: % 6.3f g    ", ax_g);
                st7735_draw_string(10, 32, textBuf, ST7735_GREEN, ST7735_BLACK, 1);

                snprintf(textBuf, sizeof(textBuf), "Y: % 6.3f g    ", ay_g);
                st7735_draw_string(10, 44, textBuf, ST7735_YELLOW, ST7735_BLACK, 1);

                snprintf(textBuf, sizeof(textBuf), "Z: % 6.3f g    ", az_g);
                st7735_draw_string(10, 56, textBuf, ST7735_BLUE, ST7735_BLACK, 1);
            }
        } else {
            ESP_LOGW(TAG, "Failed to read accelerometer data: %s", esp_err_to_name(err));
            if (display_ok) {
                st7735_draw_string(10, 32, "Read Error!      ", ST7735_RED, ST7735_BLACK, 1);
                st7735_draw_string(10, 44, "                 ", ST7735_BLACK, ST7735_BLACK, 1);
                st7735_draw_string(10, 56, "                 ", ST7735_BLACK, ST7735_BLACK, 1);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(200));
    }

    // Clean up
    mpu6050_free(busHandle, sensorHandle);
}
