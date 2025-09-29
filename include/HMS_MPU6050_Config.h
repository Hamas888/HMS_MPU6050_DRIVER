/*
  ====================================================================================================
  * File:        HMS_MPU6050_Config.h
  * Author:      Hamas Saeed
  * Version:     Rev_1.0.0
  * Date:        Sep 23 2025
  * Brief:       This Package Provide MPU6050 Driver Selection
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

#ifndef HMS_MPU6050_CONFIG_H
#define HMS_MPU6050_CONFIG_H

/*
  ┌─────────────────────────────────────────────────────────────────────┐
  │ Note:     Enable only if ChronoLog is included                      │
  │ Requires: ChronoLog library → https://github.com/Hamas888/ChronoLog │
  └─────────────────────────────────────────────────────────────────────┘
*/
#define HMS_MPU6050_DEBUG_ENABLED           0                            // Enable debug messages (1=enabled, 0=disabled)


#define HMS_MPU6050_DEVICE_NAME             "MPU6050"                    // Device Name
#define HMS_MPU6050_DEVICE_ADDR             0x68                         // MPU6050 default i2c address w/ AD0 high
#define HMS_MPU6050_DEVICE_ID               0x68                         // The correct MPU6050_WHO_AM_I value

#define HMS_MPU6050_SELF_TEST_X             0x0D                         // Self test factory calibrated values register
#define HMS_MPU6050_SELF_TEST_Y             0x0E                         // Self test factory calibrated values register
#define HMS_MPU6050_SELF_TEST_Z             0x0F                         // Self test factory calibrated values register
#define HMS_MPU6050_SELF_TEST_A             0x10                         // Self test factory calibrated values register

#define HMS_MPU6050_SMPLRT_DIV              0x19                         // sample rate divisor register
#define HMS_MPU6050_CONFIG                  0x1A                         // General configuration register
#define HMS_MPU6050_GYRO_CONFIG             0x1B                         // Gyro specfic configuration register
#define HMS_MPU6050_ACCEL_CONFIG            0x1C                         // Accelerometer specific configration register
#define HMS_MPU6050_INT_PIN_CONFIG          0x37                         // Interrupt pin configuration register
#define HMS_MPU6050_INT_ENABLE              0x38                         // Interrupt enable configuration register
#define HMS_MPU6050_INT_STATUS              0x3A                         // Interrupt status register
#define HMS_MPU6050_WHO_AM_I                0x75                         // Divice ID register
#define HMS_MPU6050_SIGNAL_PATH_RESET       0x68                         // Signal path reset register
#define HMS_MPU6050_USER_CTRL               0x6A                         // FIFO and I2C Master control register
#define HMS_MPU6050_PWR_MGMT_1              0x6B                         // Primary power/sleep control register
#define HMS_MPU6050_PWR_MGMT_2              0x6C                         // Secondary power/sleep control register

#define HMS_MPU6050_TEMP_H                  0x41                         // Temperature data high byte register
#define HMS_MPU6050_TEMP_L                  0x42                         // Temperature data low byte register

#define HMS_MPU6050_ACCEL_OUT               0x3B                         // base address for sensor data reads
#define HMS_MPU6050_MOT_THR                 0x1F                         // Motion detection threshold bits [7:0]
#define HMS_MPU6050_MOT_DUR                 0x20                         // Duration counter threshold for motion int. 1 kHz rate, LSB = 1 ms

#endif // HMS_MPU6050_CONFIG_H