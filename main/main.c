#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_mac.h"
#include "esp_http_server.h"
#include "mpu6050.h"
#include "st7735.h"
#include "hc_sr04.h"
#include "web_page.h"
#include "led_strip.h"
#include "esp_sleep.h"

static const char *TAG = "MAIN";

// I2C Pin Configuration (Accelerometer)
#define I2C_SDA_PIN     6
#define I2C_SCL_PIN     7

// TFT SPI Pin Configuration
#define PIN_MOSI        19
#define PIN_CLK         21
#define PIN_CS          22
#define PIN_DC          2
#define PIN_RST         3
#define PIN_BL          15

// HC-SR04 Pin Configuration (Distance Sensor)
#define HC_SR04_TRIG_PIN 4
#define HC_SR04_ECHO_PIN 5

// Sleep Button Pin Configuration (GPIO 1 is LP_GPIO, supporting Deep Sleep wakeup)
#define BUTTON_GPIO_PIN 1

// Motor GPIO Configuration (Left = 10, Right = 11)
#define MOTOR_LEFT_PIN  10
#define MOTOR_RIGHT_PIN 11

// Onboard RGB LED (WS2812) Config
#define LED_STRIP_GPIO_PIN 8
#define LED_STRIP_NUM_LEDS 1
static led_strip_handle_t s_led_strip = NULL;
static volatile bool s_go_to_sleep = false;
static volatile uint32_t s_last_command_time = 0;
static volatile bool s_ignore_button_until_high = false;

static void button_poll_task(void *pvParameters)
{
    while (1) {
        if (s_ignore_button_until_high) {
            // Ignore button presses until the button is released (goes back to 1)
            if (gpio_get_level(BUTTON_GPIO_PIN) == 1) {
                s_ignore_button_until_high = false;
            }
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        if (gpio_get_level(BUTTON_GPIO_PIN) == 0) {
            vTaskDelay(pdMS_TO_TICKS(30)); // Debounce
            if (!s_ignore_button_until_high && gpio_get_level(BUTTON_GPIO_PIN) == 0) {
                // Signal to main task to perform shutdown
                s_go_to_sleep = true;
                vTaskSuspend(NULL); // Suspend ourselves
            }
        }
        vTaskDelay(pdMS_TO_TICKS(20)); // Poll every 20 ms
    }
}

static void init_onboard_led(void)
{
    led_strip_config_t strip_config = {
        .strip_gpio_num = LED_STRIP_GPIO_PIN,
        .max_leds = LED_STRIP_NUM_LEDS,
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,
        .led_model = LED_MODEL_WS2812,
        .flags.invert_out = false,
    };
    led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10 * 1000 * 1000, // 10MHz
        .flags.with_dma = false,
    };
    esp_err_t err = led_strip_new_rmt_device(&strip_config, &rmt_config, &s_led_strip);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Onboard RGB LED initialized. Setting color to Green.");
        led_strip_set_pixel(s_led_strip, 0, 0, 32, 0); // Green (further reduced intensity)
        led_strip_refresh(s_led_strip);
    } else {
        ESP_LOGE(TAG, "Failed to initialize onboard RGB LED: %s", esp_err_to_name(err));
    }
}

// Wi-Fi SoftAP Configuration
#define WIFI_SSID       "ASE-remote-car"
#define WIFI_PASS       "supersafepassword"
#define WIFI_CHANNEL    1
#define MAX_STA_CONN    4

// Shared sensor readings (atomic 32-bit floats, no mutex required)
static volatile float s_ax_g = 0.0f;
static volatile float s_ay_g = 0.0f;
static volatile float s_az_g = 0.0f;
static volatile float s_distance_cm = -1.0f;

// Wi-Fi Event Handler
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data)
{
    if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
        ESP_LOGI(TAG, "Station "MACSTR" connected, AID=%d", MAC2STR(event->mac), event->aid);
    } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
        ESP_LOGI(TAG, "Station "MACSTR" disconnected, AID=%d, reason=%d", 
                 MAC2STR(event->mac), event->aid, event->reason);
    }
}

// Initialize Wi-Fi in SoftAP mode
static void wifi_init_softap(void)
{
    ESP_LOGI(TAG, "Initializing Wi-Fi SoftAP...");
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = WIFI_SSID,
            .ssid_len = strlen(WIFI_SSID),
            .channel = WIFI_CHANNEL,
            .password = WIFI_PASS,
            .max_connection = MAX_STA_CONN,
            .authmode = WIFI_AUTH_WPA2_PSK,
            .pmf_cfg = {
                .required = true,
            },
        },
    };
    
    if (strlen(WIFI_PASS) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "SoftAP started successfully. SSID: %s | Pass: %s | IP: 192.168.4.1", 
             WIFI_SSID, WIFI_PASS);
}

// HTTP GET handler for root (Dashboard HTML Page)
static esp_err_t root_get_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, INDEX_HTML, HTTPD_RESP_USE_STRLEN);
}

// HTTP GET handler for JSON accelerometer data
static esp_err_t data_get_handler(httpd_req_t *req)
{
    float ax = s_ax_g;
    float ay = s_ay_g;
    float az = s_az_g;
    float dist = s_distance_cm;
    
    // Log values every 2 seconds for debugging
    static uint32_t last_log_time = 0;
    uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
    if (now - last_log_time > 2000) {
        last_log_time = now;
        ESP_LOGI(TAG, "GET /data output: x=%.3f, y=%.3f, z=%.3f, dist=%.1f", ax, ay, az, dist);
    }

    char json[128];
    snprintf(json, sizeof(json), "{\"x\":%.3f,\"y\":%.3f,\"z\":%.3f,\"dist\":%.1f}", ax, ay, az, dist);
    
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
    httpd_resp_set_hdr(req, "Pragma", "no-cache");
    
    return httpd_resp_sendstr(req, json);
}

// HTTP GET handler for motor commands
static esp_err_t motor_get_handler(httpd_req_t *req)
{
    s_last_command_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    char* buf = NULL;
    size_t query_len = httpd_req_get_url_query_len(req);
    if (query_len > 0) {
        buf = malloc(query_len + 1);
        if (httpd_req_get_url_query_str(req, buf, query_len + 1) == ESP_OK) {
            char dir[16] = {0};
            if (httpd_query_key_value(buf, "dir", dir, sizeof(dir)) == ESP_OK) {
                if (strcmp(dir, "fwd") == 0) {
                    gpio_set_level(MOTOR_LEFT_PIN, 1);
                    gpio_set_level(MOTOR_RIGHT_PIN, 1);
                } else if (strcmp(dir, "left") == 0) {
                    gpio_set_level(MOTOR_LEFT_PIN, 1);
                    gpio_set_level(MOTOR_RIGHT_PIN, 0);
                } else if (strcmp(dir, "right") == 0) {
                    gpio_set_level(MOTOR_LEFT_PIN, 0);
                    gpio_set_level(MOTOR_RIGHT_PIN, 1);
                } else {
                    gpio_set_level(MOTOR_LEFT_PIN, 0);
                    gpio_set_level(MOTOR_RIGHT_PIN, 0);
                }
                ESP_LOGI(TAG, ">>> Motor Command: %-5s | Left (GPIO 10): %d | Right (GPIO 11): %d", 
                         dir, gpio_get_level(MOTOR_LEFT_PIN), gpio_get_level(MOTOR_RIGHT_PIN));
            }
        }
        free(buf);
    } else {
        gpio_set_level(MOTOR_LEFT_PIN, 0);
        gpio_set_level(MOTOR_RIGHT_PIN, 0);
        ESP_LOGI(TAG, ">>> Motor Command: STOP  | Left (GPIO 10): 0 | Right (GPIO 11): 0");
    }

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_sendstr(req, "{\"status\":\"ok\"}");
}

// Initialize Web Server
static httpd_handle_t start_webserver(void)
{
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.stack_size = 8192;

    ESP_LOGI(TAG, "Starting HTTP server on port %d...", config.server_port);
    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_uri_t root_uri = {
            .uri      = "/",
            .method   = HTTP_GET,
            .handler  = root_get_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server, &root_uri);

        httpd_uri_t data_uri = {
            .uri      = "/data",
            .method   = HTTP_GET,
            .handler  = data_get_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server, &data_uri);

        httpd_uri_t motor_uri = {
            .uri      = "/motor",
            .method   = HTTP_GET,
            .handler  = motor_get_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server, &motor_uri);

        return server;
    }

    ESP_LOGE(TAG, "Failed to start HTTP server!");
    return NULL;
}

void app_main(void)
{
    // 1. Initialize NVS (Required for Wi-Fi configurations)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initialize command activity timer
    s_last_command_time = xTaskGetTickCount() * portTICK_PERIOD_MS;

    // Onboard RGB LED initialization (Active = Green)
    init_onboard_led();

    // Initialize sleep button on GPIO 1
    gpio_config_t btn_config = {
        .pin_bit_mask = (1ULL << BUTTON_GPIO_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&btn_config);

    // Initialize motor control GPIOs (10 and 11)
    gpio_config_t motor_cfg = {
        .pin_bit_mask = (1ULL << MOTOR_LEFT_PIN) | (1ULL << MOTOR_RIGHT_PIN),
        .mode = GPIO_MODE_INPUT_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&motor_cfg);
    gpio_set_level(MOTOR_LEFT_PIN, 0);
    gpio_set_level(MOTOR_RIGHT_PIN, 0);

    // Create responsive button polling task (20ms interval)
    xTaskCreate(button_poll_task, "btn_poll_task", 2560, NULL, 5, NULL);

    // 2. Initialize TFT display
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
        
        // Print layout and headers
        st7735_draw_string(10, 8,  "ESP32-C6 ACCEL", ST7735_CYAN, ST7735_BLACK, 1);
        st7735_draw_string(10, 18, "==============", ST7735_GRAY, ST7735_BLACK, 1);
    } else {
        ESP_LOGE(TAG, "Failed to initialize TFT display!");
    }

    // 3. Initialize Wi-Fi SoftAP
    wifi_init_softap();

    // 4. Start Web Server
    start_webserver();

    // 5. Initialize HC-SR04 Distance Sensor
    ESP_LOGI(TAG, "Initializing HC-SR04 sensor (Trig: GPIO %d, Echo: GPIO %d)...", 
             HC_SR04_TRIG_PIN, HC_SR04_ECHO_PIN);
    hc_sr04_init(HC_SR04_TRIG_PIN, HC_SR04_ECHO_PIN);

    // 6. Initialize MPU-6050 Accelerometer (Robust retry initialization)
    i2c_master_bus_handle_t busHandle = NULL;
    i2c_master_dev_handle_t sensorHandle = NULL;
    bool mpu_ok = false;

    ESP_LOGI(TAG, "Initializing MPU-6050 sensor on SDA: GPIO %d, SCL: GPIO %d", I2C_SDA_PIN, I2C_SCL_PIN);
    esp_err_t err = mpu6050_init(&busHandle, &sensorHandle, MPU6050_SENSOR_ADDR, 
                                 I2C_SDA_PIN, I2C_SCL_PIN, MPU6050_SCL_DFLT_FREQ_HZ);
    if (err == ESP_OK) {
        mpu_ok = true;
        ESP_LOGI(TAG, "MPU-6050 initialized successfully.");
    } else {
        ESP_LOGE(TAG, "Failed to initialize MPU-6050 accelerometer: %s", esp_err_to_name(err));
        if (display_ok) {
            st7735_draw_string(10, 32, "Sensor Init Err", ST7735_RED, ST7735_BLACK, 1);
        }
    }

    ESP_LOGI(TAG, "Entering sensor reading and display loop...");

    int16_t ax = 0, ay = 0, az = 0;
    float dist_cm = -1.0f;
    char textBuf[32];
    
    while (1) {
        if (s_go_to_sleep) {
            ESP_LOGI(TAG, "Sleep triggered. Safely entering Deep Sleep from main task...");

            // Set RGB LED to Red (low brightness: 32)
            if (s_led_strip != NULL) {
                led_strip_set_pixel(s_led_strip, 0, 32, 0, 0); // Red
                led_strip_refresh(s_led_strip);
            }

            // Clear screen and draw deep sleep message
            if (display_ok) {
                st7735_fill_screen(ST7735_BLACK);
                st7735_draw_string(10, 30, "Deep Sleep Mode", ST7735_RED, ST7735_BLACK, 1);
            }

            // Delay to let RMT and TFT finish transmitting/refreshing
            vTaskDelay(pdMS_TO_TICKS(100));

            // Configure Deep Sleep wakeup using EXT1 on GPIO 1 (active low)
            esp_sleep_enable_ext1_wakeup(1ULL << BUTTON_GPIO_PIN, ESP_EXT1_WAKEUP_ANY_LOW);

            // Keep RTC peripherals powered on during deep sleep so the internal pull-up is maintained
            esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
            gpio_pullup_en(BUTTON_GPIO_PIN);
            gpio_pulldown_dis(BUTTON_GPIO_PIN);

            // Enter Deep Sleep
            esp_deep_sleep_start();
        }

        // Check inactivity timeout (60 seconds) for Light Sleep
        uint32_t current_ticks = xTaskGetTickCount();
        if (current_ticks * portTICK_PERIOD_MS - s_last_command_time > 60000) {
            ESP_LOGI(TAG, "No movement command for 60s. Entering Light Sleep...");

            // 1. Set RGB LED to Blue (low brightness: 32)
            if (s_led_strip != NULL) {
                led_strip_set_pixel(s_led_strip, 0, 0, 0, 32); // Blue
                led_strip_refresh(s_led_strip);
            }

            // 2. Clear screen and draw light sleep message
            if (display_ok) {
                st7735_fill_screen(ST7735_BLACK);
                st7735_draw_string(10, 30, "Light Sleep Mode", ST7735_BLUE, ST7735_BLACK, 1);
            }

            // 3. Delay to let RMT and TFT finish refreshing
            vTaskDelay(pdMS_TO_TICKS(100));

            // 4. Configure GPIO 1 (button) to wake us up from Light Sleep
            gpio_wakeup_enable(BUTTON_GPIO_PIN, GPIO_INTR_LOW_LEVEL);
            esp_sleep_enable_gpio_wakeup();

            // Ignore the wake up button press until it is released
            s_ignore_button_until_high = true;

            // 5. Enter Light Sleep
            esp_light_sleep_start();

            // --- WOKEN UP FROM LIGHT SLEEP ---
            ESP_LOGI(TAG, "Woke up from Light Sleep!");
            s_ignore_button_until_high = true; // Block deep sleep trigger until button release

            // 6. Restore LED to Green (low brightness: 32)
            if (s_led_strip != NULL) {
                led_strip_set_pixel(s_led_strip, 0, 0, 32, 0); // Green
                led_strip_refresh(s_led_strip);
            }

            // 7. Restore TFT screen display headers
            if (display_ok) {
                st7735_fill_screen(ST7735_BLACK);
                st7735_draw_string(10, 8,  "ESP32-C6 ACCEL", ST7735_CYAN, ST7735_BLACK, 1);
                st7735_draw_string(10, 18, "==============", ST7735_GRAY, ST7735_BLACK, 1);
            }

            // 8. Reset inactivity timer
            s_last_command_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
        }

        // Robust MPU-6050 retry initialization if it was not successful initially
        if (!mpu_ok) {
            static uint32_t last_retry_ticks = 0;
            uint32_t now_ticks = xTaskGetTickCount();
            if (now_ticks - last_retry_ticks > pdMS_TO_TICKS(5000)) { // retry every 5s
                last_retry_ticks = now_ticks;
                ESP_LOGI(TAG, "Retrying MPU-6050 initialization...");
                err = mpu6050_init(&busHandle, &sensorHandle, MPU6050_SENSOR_ADDR, 
                                   I2C_SDA_PIN, I2C_SCL_PIN, MPU6050_SCL_DFLT_FREQ_HZ);
                if (err == ESP_OK) {
                    mpu_ok = true;
                    ESP_LOGI(TAG, "MPU-6050 initialized successfully on retry.");
                }
            }
        }

        double ax_g = 0.0, ay_g = 0.0, az_g = 0.0;
        if (mpu_ok && sensorHandle != NULL) {
            err = mpu6050_read_accel(sensorHandle, &ax, &ay, &az);
            if (err == ESP_OK) {
                ax_g = (double)ax / 16384.0;
                ay_g = (double)ay / 16384.0;
                az_g = (double)az / 16384.0;

                ESP_LOGI(TAG, "Accel Raw: X=%6d | Y=%6d | Z=%6d  ==>  G: X=%6.3f | Y=%6.3f | Z=%6.3f g",
                         ax, ay, az, ax_g, ay_g, az_g);

                s_ax_g = ax_g;
                s_ay_g = ay_g;
                s_az_g = az_g;
            } else {
                ESP_LOGW(TAG, "Failed to read accelerometer data: %s", esp_err_to_name(err));
                
                // If it fails repeatedly, mark as disconnected to trigger retry logic
                static int fail_count = 0;
                fail_count++;
                if (fail_count > 5) {
                    ESP_LOGW(TAG, "MPU-6050 failed repeatedly. Resetting connection.");
                    mpu_ok = false;
                    fail_count = 0;
                    mpu6050_free(busHandle, sensorHandle);
                    busHandle = NULL;
                    sensorHandle = NULL;
                }
            }
        } else {
            s_ax_g = 0.0f;
            s_ay_g = 0.0f;
            s_az_g = 0.0f;
        }

        // Read distance data
        esp_err_t dist_err = hc_sr04_read_distance(HC_SR04_TRIG_PIN, HC_SR04_ECHO_PIN, &dist_cm);
        if (dist_err == ESP_OK) {
            ESP_LOGI(TAG, "Distance: %.1f cm", dist_cm);
            s_distance_cm = dist_cm;
        } else {
            ESP_LOGW(TAG, "Failed to read distance data: %s", esp_err_to_name(dist_err));
            s_distance_cm = -1.0f;
        }

        // Update local display values in a clean, compact format
        if (display_ok) {
            if (mpu_ok) {
                snprintf(textBuf, sizeof(textBuf), "X:% 5.2f  Y:% 5.2f", ax_g, ay_g);
                st7735_draw_string(10, 32, textBuf, ST7735_GREEN, ST7735_BLACK, 1);

                if (s_distance_cm >= 0.0f) {
                    snprintf(textBuf, sizeof(textBuf), "Z:% 5.2f  D:% 4.1f", az_g, s_distance_cm);
                } else {
                    snprintf(textBuf, sizeof(textBuf), "Z:% 5.2f  D: --- ", az_g);
                }
                st7735_draw_string(10, 44, textBuf, ST7735_BLUE, ST7735_BLACK, 1);
            } else {
                st7735_draw_string(10, 32, "Sensor Conn Err", ST7735_RED, ST7735_BLACK, 1);
                if (s_distance_cm >= 0.0f) {
                    snprintf(textBuf, sizeof(textBuf), "Z: ---   D:% 4.1f", s_distance_cm);
                } else {
                    snprintf(textBuf, sizeof(textBuf), "Z: ---   D: --- ");
                }
                st7735_draw_string(10, 44, textBuf, ST7735_BLUE, ST7735_BLACK, 1);
            }

            // Draw motor status on the display (perfect for testing without real motors connected)
            int left = gpio_get_level(MOTOR_LEFT_PIN);
            int right = gpio_get_level(MOTOR_RIGHT_PIN);
            if (left && right) {
                st7735_draw_string(10, 56, "Motors: FORWARD", ST7735_GREEN, ST7735_BLACK, 1);
            } else if (left && !right) {
                st7735_draw_string(10, 56, "Motors: LEFT   ", ST7735_YELLOW, ST7735_BLACK, 1);
            } else if (!left && right) {
                st7735_draw_string(10, 56, "Motors: RIGHT  ", ST7735_BLUE, ST7735_BLACK, 1);
            } else {
                st7735_draw_string(10, 56, "Motors: STOP   ", ST7735_GRAY, ST7735_BLACK, 1);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(200));
    }

    // Clean up
    mpu6050_free(busHandle, sensorHandle);
}
