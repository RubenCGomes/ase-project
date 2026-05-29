#ifndef __HC_SR04_H__INCLUDED__
#define __HC_SR04_H__INCLUDED__

#include "driver/gpio.h"
#include "esp_err.h"

/**
 * @brief Initialize HC-SR04 distance sensor.
 * 
 * @param trigPin GPIO number for Trigger pin (Output)
 * @param echoPin GPIO number for Echo pin (Input)
 */
void hc_sr04_init(int trigPin, int echoPin);

/**
 * @brief Trigger a measurement and read the distance in centimeters.
 * 
 * @param trigPin GPIO number for Trigger pin
 * @param echoPin GPIO number for Echo pin
 * @param pDistanceCm Pointer to store the measured distance in cm
 * @return esp_err_t ESP_OK on success, ESP_ERR_TIMEOUT if echo pulse timed out
 */
esp_err_t hc_sr04_read_distance(int trigPin, int echoPin, float* pDistanceCm);

#endif // __HC_SR04_H__INCLUDED__
