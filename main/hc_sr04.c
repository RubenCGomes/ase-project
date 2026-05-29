#include "hc_sr04.h"
#include "esp_timer.h"
#include "rom/ets_sys.h" // For ets_delay_us
#include "esp_log.h"

void hc_sr04_init(int trigPin, int echoPin)
{
    // Reset and configure Trig pin as Output with pull-down
    gpio_reset_pin(trigPin);
    gpio_set_direction(trigPin, GPIO_MODE_OUTPUT);
    gpio_set_pull_mode(trigPin, GPIO_PULLDOWN_ONLY);
    gpio_set_level(trigPin, 0);

    // Reset and configure Echo pin as Input (floating, since we use a voltage divider)
    gpio_reset_pin(echoPin);
    gpio_set_direction(echoPin, GPIO_MODE_INPUT);
    gpio_set_pull_mode(echoPin, GPIO_FLOATING);
}

esp_err_t hc_sr04_read_distance(int trigPin, int echoPin, float* pDistanceCm)
{
    if (pDistanceCm == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // 1. Ensure Trig is LOW first
    gpio_set_level(trigPin, 0);
    ets_delay_us(5);

    // 2. Trigger the sensor by sending a 15us HIGH pulse (ensuring all clones trigger)
    gpio_set_level(trigPin, 1);
    ets_delay_us(15);
    gpio_set_level(trigPin, 0);

    // 3. Wait for Echo to go HIGH with timeout (30ms)
    int64_t start_time = esp_timer_get_time();
    while (gpio_get_level(echoPin) == 0) {
        if (esp_timer_get_time() - start_time > 30000) {
            ESP_LOGW("HC_SR04", "Timeout waiting for Echo to go HIGH! Pin state: %d", gpio_get_level(echoPin));
            return ESP_ERR_TIMEOUT;
        }
    }
    int64_t echo_start = esp_timer_get_time();

    // 4. Wait for Echo to go LOW with timeout (50ms)
    while (gpio_get_level(echoPin) == 1) {
        if (esp_timer_get_time() - echo_start > 50000) {
            *pDistanceCm = 400.0f; // Out of range / no obstacle
            return ESP_OK;
        }
    }
    int64_t echo_end = esp_timer_get_time();

    // 5. Calculate distance
    int64_t duration = echo_end - echo_start; // duration in microseconds
    
    // Distance = duration * speed of sound (343 m/s = 0.0343 cm/us) / 2
    *pDistanceCm = (float)duration * 0.0343f / 2.0f;
    
    // Cap at typical max sensor range
    if (*pDistanceCm > 400.0f) {
        *pDistanceCm = 400.0f;
    }
    
    return ESP_OK;
}
