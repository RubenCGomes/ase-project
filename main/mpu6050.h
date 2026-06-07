#ifndef __MPU6050_H__INCLUDED__
#define __MPU6050_H__INCLUDED__

#include <stdint.h>
#include <stdbool.h>
#include "driver/i2c_master.h"

#define MPU6050_SENSOR_ADDR         0x68
#define MPU6050_SCL_DFLT_FREQ_HZ    100000 // 100 kHz

/**
 * @brief
 *
 * @param pBusHandle Pointer to I2C master bus handle
 * @param pSensorHandle Pointer to I2C device handle for the sensor
 * @param sensorAddr 7-bit I2C address of MPU-6050
 * @param sdaPin GPIO number for SDA
 * @param sclPin GPIO number for SCL
 * @param clkSpeedHz I2C clock frequency
 * @return 
 */
esp_err_t mpu6050_init(i2c_master_bus_handle_t* pBusHandle,
                       i2c_master_dev_handle_t* pSensorHandle,
                       uint8_t sensorAddr, int sdaPin, int sclPin, uint32_t clkSpeedHz);

/**
 * @brief 
 *
 * @param busHandle I2C master bus handle
 * @param sensorHandle I2C device handle
 * @return 
 */
esp_err_t mpu6050_free(i2c_master_bus_handle_t busHandle,
                       i2c_master_dev_handle_t sensorHandle);

/**
 * @brief 
 *
 * @param sensorHandle I2C device handle for the MPU-6050 sensor
 * @param ax Pointer to store raw X axis acceleration
 * @param ay Pointer to store raw Y axis acceleration
 * @param az Pointer to store raw Z axis acceleration
 * @return 
 */
esp_err_t mpu6050_read_accel(i2c_master_dev_handle_t sensorHandle,
                             int16_t* ax, int16_t* ay, int16_t* az);

#endif
