#include "mpu6050.h"
#include "esp_log.h"

static const char *TAG = "MPU6050";

#define MPU6050_REG_PWR_MGMT_1      0x6B
#define MPU6050_REG_WHO_AM_I        0x75
#define MPU6050_REG_ACCEL_XOUT_H    0x3B

esp_err_t mpu6050_init(i2c_master_bus_handle_t* pBusHandle,
                       i2c_master_dev_handle_t* pSensorHandle,
                       uint8_t sensorAddr, int sdaPin, int sclPin, uint32_t clkSpeedHz)
{
    if (pBusHandle == NULL || pSensorHandle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "Initializing I2C master bus...");
    i2c_master_bus_config_t i2cMasterCfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .scl_io_num = sclPin,
        .sda_io_num = sdaPin,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    esp_err_t err = i2c_new_master_bus(&i2cMasterCfg, pBusHandle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create I2C master bus: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "Adding MPU-6050 device at address 0x%02X...", sensorAddr);
    i2c_device_config_t i2cDevCfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = sensorAddr,
        .scl_speed_hz = clkSpeedHz,
    };

    err = i2c_master_bus_add_device(*pBusHandle, &i2cDevCfg, pSensorHandle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add device to I2C bus: %s", esp_err_to_name(err));
        i2c_del_master_bus(*pBusHandle);
        return err;
    }

    // Wake up MPU-6050 by writing 0 to PWR_MGMT_1
    ESP_LOGI(TAG, "Waking up MPU-6050...");
    uint8_t wakeupCmd[2] = {MPU6050_REG_PWR_MGMT_1, 0x00};
    err = i2c_master_transmit(*pSensorHandle, wakeupCmd, sizeof(wakeupCmd), -1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write wake up command: %s", esp_err_to_name(err));
        i2c_master_bus_rm_device(*pSensorHandle);
        i2c_del_master_bus(*pBusHandle);
        return err;
    }

    // Verify WHO_AM_I register
    uint8_t whoAmIReg = MPU6050_REG_WHO_AM_I;
    uint8_t whoAmIVal = 0;
    err = i2c_master_transmit_receive(*pSensorHandle, &whoAmIReg, 1, &whoAmIVal, 1, -1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read WHO_AM_I register: %s", esp_err_to_name(err));
        i2c_master_bus_rm_device(*pSensorHandle);
        i2c_del_master_bus(*pBusHandle);
        return err;
    }

    ESP_LOGI(TAG, "WHO_AM_I read: 0x%02X (Expected: 0x68)", whoAmIVal);
    if (whoAmIVal != 0x68) {
        ESP_LOGW(TAG, "WHO_AM_I value mismatch! Got 0x%02X, expected 0x68", whoAmIVal);
        // We can still proceed, but log it as a warning
    }

    ESP_LOGI(TAG, "MPU-6050 initialized successfully.");
    return ESP_OK;
}

esp_err_t mpu6050_free(i2c_master_bus_handle_t busHandle, i2c_master_dev_handle_t sensorHandle)
{
    ESP_LOGI(TAG, "Cleaning up MPU-6050 device and I2C master bus...");
    esp_err_t err1 = i2c_master_bus_rm_device(sensorHandle);
    esp_err_t err2 = i2c_del_master_bus(busHandle);

    if (err1 != ESP_OK) return err1;
    return err2;
}

esp_err_t mpu6050_read_accel(i2c_master_dev_handle_t sensorHandle, int16_t* ax, int16_t* ay, int16_t* az)
{
    if (ax == NULL || ay == NULL || az == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t regAddr = MPU6050_REG_ACCEL_XOUT_H;
    uint8_t data[6] = {0};

    esp_err_t err = i2c_master_transmit_receive(sensorHandle, &regAddr, 1, data, sizeof(data), -1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read accelerometer registers: %s", esp_err_to_name(err));
        return err;
    }

    *ax = (int16_t)((data[0] << 8) | data[1]);
    *ay = (int16_t)((data[2] << 8) | data[3]);
    *az = (int16_t)((data[4] << 8) | data[5]);

    return ESP_OK;
}
