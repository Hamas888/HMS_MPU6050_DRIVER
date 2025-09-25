   /*
 ====================================================================================================
 * File:        HMS_MPU6050_DRIVER.h
 * Author:      Hamas Saeed
 * Version:     Rev_1.0.0
 * Date:        Sep 25 2025
 * Brief:       This Package Provide MPU6050 Driver Library for Cross Platform (STM/ESP/nRF)
 * 
 ====================================================================================================
 * License: 
 * MIT License
 * 
 * Copyright (c) 2025 Hamas Saeed
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 * 
 * For any inquiries, contact Hamas Saeed at hamasaeed@gmail.com
 *
 ====================================================================================================
 */

#ifndef HMS_MPU6050_DRIVER_H
#define HMS_MPU6050_DRIVER_H

#if defined(ARDUINO)                                                                                       // Platform detection
  #define HMS_MPU6050_PLATFORM_ARDUINO
#elif defined(ESP_PLATFORM)
  #define HMS_MPU6050_PLATFORM_ESP_IDF
#elif defined(__ZEPHYR__)
  #define HMS_MPU6050_PLATFORM_ZEPHYR
#elif defined(STM32F0) || defined(STM32F1) || defined(STM32F3) || defined(STM32F4) || \
      defined(STM32F7) || defined(STM32G0) || defined(STM32G4) || defined(STM32H7) || \
      defined(STM32L0) || defined(STM32L1) || defined(STM32L4) || defined(STM32L5) || \
      defined(STM32WB) || defined(STM32WL)
  #define HMS_MPU6050_PLATFORM_STM32_HAL
#endif

#if defined(HMS_MPU6050_PLATFORM_ARDUINO)
  #include <Wire.h>
  #include <Arduino.h>
#elif defined(HMS_MPU6050_PLATFORM_ESP_IDF)
#elif defined(HMS_MPU6050_PLATFORM_ZEPHYR)
  #include <stdio.h>
  #include <zephyr/device.h>
  #include <zephyr/drivers/i2c.h>
#elif defined(HMS_MPU6050_PLATFORM_STM32_HAL)
#endif

#endif // HMS_MPU6050_DRIVER_H

#include "HMS_MPU6050_Config.h"

#if defined(HMS_MPU6050_DEBUG_ENABLED) && (HMS_MPU6050_DEBUG_ENABLED == 1)
  #define HMS_MPU6050_LOGGER_ENABLED
#endif

typedef enum {
    HMS_MPU6050_OK       = 0x00,
    HMS_MPU6050_ERROR    = 0x01,
    HMS_MPU6050_BUSY     = 0x02,
    HMS_MPU6050_TIMEOUT  = 0x03,
    HMS_MPU6050_NOT_FOUND= 0x04
} HMS_MPU6050_StatusTypeDef;

typedef enum {
  HMS_MPU6050_FSYNC_OUT_DISABLED,
  HMS_MPU6050_FSYNC_OUT_TEMP,
  HMS_MPU6050_FSYNC_OUT_GYROX,
  HMS_MPU6050_FSYNC_OUT_GYROY,
  HMS_MPU6050_FSYNC_OUT_GYROZ,
  HMS_MPU6050_FSYNC_OUT_ACCELX,
  HMS_MPU6050_FSYNC_OUT_ACCELY,
  HMS_MPU6050_FSYNC_OUT_ACCEL_Z,
} HMS_MPU6050_FsyncOut;

typedef enum {
  HMS_MPU6050_INTR_8MHz,
  HMS_MPU6050_PLL_GYROX,
  HMS_MPU6050_PLL_GYROY,
  HMS_MPU6050_PLL_GYROZ,
  HMS_MPU6050_PLL_EXT_32K,
  HMS_MPU6050_PLL_EXT_19MHz,
  HMS_MPU6050_STOP = 7,
} HMS_MPU6050_ClockSelect;

typedef enum {
  HMS_MPU6050_RANGE_2_G     = 0b00,                             // +/- 2g (default value)
  HMS_MPU6050_RANGE_4_G     = 0b01,                             // +/- 4g
  HMS_MPU6050_RANGE_8_G     = 0b10,                             // +/- 8g
  HMS_MPU6050_RANGE_16_G    = 0b11,                             // +/- 16g
} HMS_MPU6050_AccelRange;


typedef enum {
  HMS_MPU6050_RANGE_250_DEG,                                    // +/- 250 deg/s (default value)
  HMS_MPU6050_RANGE_500_DEG,                                    // +/- 500 deg/s
  HMS_MPU6050_RANGE_1000_DEG,                                   // +/- 1000 deg/s
  HMS_MPU6050_RANGE_2000_DEG,                                   // +/- 2000 deg/s
} HMS_MPU6050_GyroRange;

typedef enum {
  HMS_MPU6050_BAND_260_HZ,                                      // Docs imply this disables the filter
  HMS_MPU6050_BAND_184_HZ,                                      // 184 Hz
  HMS_MPU6050_BAND_94_HZ,                                       // 94 Hz
  HMS_MPU6050_BAND_44_HZ,                                       // 44 Hz
  HMS_MPU6050_BAND_21_HZ,                                       // 21 Hz
  HMS_MPU6050_BAND_10_HZ,                                       // 10 Hz
  HMS_MPU6050_BAND_5_HZ,                                        // 5 Hz
} HMS_MPU6050_BandwidthTypeDef;

typedef enum {
  HMS_MPU6050_HIGHPASS_DISABLE,
  HMS_MPU6050_HIGHPASS_5_HZ,
  HMS_MPU6050_HIGHPASS_2_5_HZ,
  HMS_MPU6050_HIGHPASS_1_25_HZ,
  HMS_MPU6050_HIGHPASS_0_63_HZ,
  HMS_MPU6050_HIGHPASS_UNUSED,
  HMS_MPU6050_HIGHPASS_HOLD,
} HMS_MPU6050_HighPassFilter;

typedef enum {
  HMS_MPU6050_CYCLE_1_25_HZ,                                    // 1.25 Hz
  HMS_MPU6050_CYCLE_5_HZ,                                       // 5 Hz
  HMS_MPU6050_CYCLE_20_HZ,                                      // 20 Hz
  HMS_MPU6050_CYCLE_40_HZ,                                      // 40 Hz
} HMS_MPU6050_CycleRate;

class HMS_MPU6050 {
public:
    HMS_MPU6050();
    ~HMS_MPU6050();

    #if defined(HMS_MPU6050_PLATFORM_ARDUINO)
        HMS_MPU6050_StatusTypeDef begin(
            TwoWire *theWire = &Wire, 
            uint8_t addr = HMS_MPU6050_DEVICE_ADDR, int32_t sensor_id = 0
        );
    #elif defined(HMS_MPU6050_PLATFORM_ESP_IDF)
        HMS_MPU6050_StatusTypeDef begin(
            i2c_port_t i2c_port = I2C_NUM_0, 
            uint8_t addr = HMS_MPU6050_DEVICE_ADDR, int32_t sensor_id = 0
        );
    #elif defined(HMS_MPU6050_PLATFORM_ZEPHYR)
        HMS_MPU6050_StatusTypeDef begin(
            const struct device *i2c_dev = NULL, 
            uint8_t addr = HMS_MPU6050_DEVICE_ADDR, int32_t sensor_id = 0
        );
    #elif defined(HMS_MPU6050_PLATFORM_STM32_HAL)
        HMS_MPU6050_StatusTypeDef begin(
            I2C_HandleTypeDef *hi2c = NULL, 
            uint8_t addr = HMS_MPU6050_DEVICE_ADDR, int32_t sensor_id = 0
        );
    #endif


private:
      bool initialized  = false;
  #if defined(HMS_MPU6050_PLATFORM_ARDUINO)
    TwoWire *mpu6050_wire = NULL;
  #elif defined(HMS_MPU6050_PLATFORM_ESP_IDF)
    i2c_port_t mpu6050_i2c_port;
  #elif defined(HMS_MPU6050_PLATFORM_ZEPHYR)
    struct device *mpu6050_i2c_dev;
  #elif defined(HMS_MPU6050_PLATFORM_STM32_HAL)
    I2C_HandleTypeDef *mpu6050_hi2c;
  #endif
};