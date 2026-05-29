idf_component_register(SRCS "main.c" "mpu6050.c" "hc_sr04.c"
                    INCLUDE_DIRS "."
                    REQUIRES st7735_driver driver esp_driver_gpio esp_http_server esp_wifi nvs_flash esp_event led_strip)
