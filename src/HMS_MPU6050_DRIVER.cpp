#include "HMS_MPU6050_DRIVER.h"

#if defined(HMS_MPU6050_LOGGER_ENABLED)
  #include "ChronoLog.h"
  ChronoLogger mpuLogger("HMS_MPU6050", CHRONOLOG_LEVEL_DEBUG);
#endif

HMS_MPU6050::HMS_MPU6050() {
    // Constructor implementation
}

HMS_MPU6050::~HMS_MPU6050() {
    // Destructor implementation
}

#if defined(HMS_MPU6050_PLATFORM_ARDUINO)
HMS_MPU6050_StatusTypeDef HMS_MPU6050::begin(TwoWire *wire, uint8_t addr) {
  mpu6050_wire = wire;
  deviceAddress = addr;
  mpu6050_wire->begin();
  mpu6050_wire->beginTransmission(deviceAddress);
  if (mpu6050_wire->endTransmission() != 0) {
    #ifdef HMS_MPU6050_LOGGER_ENABLED
      mpuLogger.error("Device not found at address 0x%02X", deviceAddress);
    #endif
    return HMS_MPU6050_NOT_FOUND;                                                         // Device not found
 }
  #ifdef HMS_MPU6050_LOGGER_ENABLED
    mpuLogger.info("Device found at address 0x%02X", deviceAddress);
  #endif
  return init();
}

uint8_t HMS_MPU6050::readRegister(uint8_t registerAddr) {
  mpu6050_wire->beginTransmission(deviceAddress);
  mpu6050_wire->write(registerAddr);
  mpu6050_wire->endTransmission(false);                                                    // Send repeated start
  mpu6050_wire->requestFrom(deviceAddress, (uint8_t)1);
  if (mpu6050_wire->available()) {
      return mpu6050_wire->read();
  }
  return 0;                                                                               // Return 0 if no data available
}

void HMS_MPU6050::writeRegister(uint8_t registerAddr, uint8_t value) {
  mpu6050_wire->beginTransmission(deviceAddress);
  mpu6050_wire->write(registerAddr);
  mpu6050_wire->write(value);
  
  #ifdef HMS_MPU6050_LOGGER_ENABLED
  uint8_t result = mpu6050_wire->endTransmission();
    if (result != 0) {
      mpuLogger.error("I2C write failed with error code: %d", result);
    }
  #else
    mpu6050_wire->endTransmission();
  #endif
}

#elif defined(HMS_MPU6050_PLATFORM_ESP_IDF)
HMS_MPU6050_StatusTypeDef HMS_MPU6050::begin(i2c_port_t i2c_port, uint8_t addr) {
    // ESP-IDF-specific initialization code here
    return HMS_MPU6050_OK;
}

#elif defined(HMS_MPU6050_PLATFORM_ZEPHYR)
HMS_MPU6050_StatusTypeDef HMS_MPU6050::begin(const struct device *i2c_dev, uint8_t addr) {
    if (i2c_dev == NULL) {
        #ifdef HMS_MPU6050_LOGGER_ENABLED
            mpuLogger.error("I2C device is NULL");
        #endif
        return HMS_MPU6050_ERROR;
    }
    mpu6050_i2c_dev = (struct device *)i2c_dev;
    deviceAddress = addr;

    if (!device_is_ready(mpu6050_i2c_dev)) {
        #ifdef HMS_MPU6050_LOGGER_ENABLED
            mpuLogger.error("Device not found at address 0x%02X", HMS_MPU6050_DEVICE_ADDR);
        #endif
        return HMS_MPU6050_ERROR;
    }
    
    uint8_t dummy = 0;
    struct i2c_msg msg = {
        .buf = &dummy,
        .len = 1,
        .flags = I2C_MSG_WRITE | I2C_MSG_STOP
    };

    int ret = i2c_transfer(i2c_dev, &msg, 1, HMS_MPU6050_DEVICE_ADDR);

    if (ret != 0) {
        return HMS_MPU6050_NOT_FOUND;                                                       // Device not found
    }

    return init();
}

uint8_t HMS_MPU6050::readRegister(uint8_t registerAddr) {
    uint8_t value = 0;
    int ret = i2c_reg_read_byte(mpu6050_i2c_dev, deviceAddress, registerAddr, &value);
    if (ret != 0) {
        return 0; // Return 0 if read failed
    }
    return value;
}

void HMS_MPU6050::writeRegister(uint8_t registerAddr, uint8_t value) {
    i2c_reg_write_byte(mpu6050_i2c_dev, deviceAddress, registerAddr, value);
}

#elif defined(HMS_MPU6050_PLATFORM_STM32_HAL)
HMS_MPU6050_StatusTypeDef HMS_MPU6050::begin(I2C_HandleTypeDef *hi2c, uint8_t addr) {
    // STM32 HAL-specific initialization code here
    return HMS_MPU6050_OK;
}

#endif

void HMS_MPU6050::mpuDelay(uint32_t ms) {
    #if defined(HMS_MPU6050_PLATFORM_ARDUINO)
        delay(ms);
    #elif defined(HMS_MPU6050_PLATFORM_ESP_IDF)
        vTaskDelay(ms / portTICK_PERIOD_MS);
    #elif defined(HMS_MPU6050_PLATFORM_ZEPHYR)
        k_msleep(ms);
    #elif defined(HMS_MPU6050_PLATFORM_STM32_HAL)
        HAL_Delay(ms);
    #endif
}

void HMS_MPU6050::reset() {
  uint8_t powerManagementValue = readRegister(HMS_MPU6050_PWR_MGMT_1);                          // Reset device by setting bit 7 of PWR_MGMT_1 register
  writeRegister(HMS_MPU6050_PWR_MGMT_1, powerManagementValue | 0x80);                           // Set bit 7 to 1

  while (readRegister(HMS_MPU6050_PWR_MGMT_1) & 0x80) {                                     // Wait for reset to complete (bit 7 should return to 0)
    mpuDelay(1);
  }
  mpuDelay(100);

  
  writeRegister(HMS_MPU6050_SIGNAL_PATH_RESET, 0x07);                                       // Reset signal path by writing 0x07 to SIGNAL_PATH_RESET register

  mpuDelay(100);
}

void HMS_MPU6050::setInterruptPinLatch(bool held) {
  uint8_t intPinConfigValue = readRegister(HMS_MPU6050_INT_PIN_CONFIG);
  
  #ifdef HMS_MPU6050_LOGGER_ENABLED
    mpuLogger.debug("INT_PIN_CONFIG before: 0x%02X", intPinConfigValue);
  #endif
  
  if (held) {
    intPinConfigValue |= 0x20;                                                               // Set bit 5 for latch until cleared
  } else {
    intPinConfigValue &= ~0x20;                                                              // Clear bit 5 for 50us pulse
  }
  writeRegister(HMS_MPU6050_INT_PIN_CONFIG, intPinConfigValue);
  
  #ifdef HMS_MPU6050_LOGGER_ENABLED
    uint8_t verifyValue = readRegister(HMS_MPU6050_INT_PIN_CONFIG);
    mpuLogger.debug("INT_PIN_CONFIG after: 0x%02X (expected: 0x%02X)", verifyValue, intPinConfigValue);
    if (verifyValue != intPinConfigValue) {
      mpuLogger.error("Interrupt pin latch write failed!");
    }
  #endif
}

void HMS_MPU6050::setSampleRateDivisor(uint8_t divisor) {
  #ifdef HMS_MPU6050_LOGGER_ENABLED
    uint8_t beforeValue = readRegister(HMS_MPU6050_SMPLRT_DIV);
    mpuLogger.debug("SMPLRT_DIV before: 0x%02X", beforeValue);
  #endif
  
  writeRegister(HMS_MPU6050_SMPLRT_DIV, divisor);
  
  // Verify the write
  #ifdef HMS_MPU6050_LOGGER_ENABLED
    uint8_t verifyValue = readRegister(HMS_MPU6050_SMPLRT_DIV);
    mpuLogger.debug("SMPLRT_DIV after: 0x%02X (expected: 0x%02X)", verifyValue, divisor);
    if (verifyValue != divisor) {
      mpuLogger.error("Sample rate divisor write failed!");
    }
  #endif
}

void HMS_MPU6050::setFilterBandwidth(HMS_MPU6050_Bandwidth bandwidth) {
  uint8_t configValue = readRegister(HMS_MPU6050_CONFIG);
  
  #ifdef HMS_MPU6050_LOGGER_ENABLED
    mpuLogger.debug("CONFIG before: 0x%02X", configValue);
  #endif
  
  configValue = (configValue & 0xF8) | (bandwidth & 0x07);                                  // Clear bits 0-2, set new bandwidth
  writeRegister(HMS_MPU6050_CONFIG, configValue);
  
  #ifdef HMS_MPU6050_LOGGER_ENABLED
    uint8_t verifyValue = readRegister(HMS_MPU6050_CONFIG);
    mpuLogger.debug("CONFIG after: 0x%02X (expected: 0x%02X)", verifyValue, configValue);
    if (verifyValue != configValue) {
      mpuLogger.error("Filter bandwidth write failed!");
    }
  #endif
}

void HMS_MPU6050::setInterruptPinPolarity(bool activeLow) {
  uint8_t intPinConfigValue = readRegister(HMS_MPU6050_INT_PIN_CONFIG);
  
  #ifdef HMS_MPU6050_LOGGER_ENABLED
    mpuLogger.debug("INT_PIN_CONFIG before: 0x%02X", intPinConfigValue);
  #endif
  
  if (activeLow) {
    intPinConfigValue |= 0x80;                                                               // Set bit 7 for active low
  } else {
    intPinConfigValue &= ~0x80;                                                              // Clear bit 7 for active high
  }
  writeRegister(HMS_MPU6050_INT_PIN_CONFIG, intPinConfigValue);
  
  #ifdef HMS_MPU6050_LOGGER_ENABLED
    uint8_t verifyValue = readRegister(HMS_MPU6050_INT_PIN_CONFIG);
    mpuLogger.debug("INT_PIN_CONFIG after: 0x%02X (expected: 0x%02X)", verifyValue, intPinConfigValue);
    if (verifyValue != intPinConfigValue) {
      mpuLogger.error("Interrupt pin polarity write failed!");
    }
  #endif
}

void HMS_MPU6050::setGyroRange(HMS_MPU6050_GyroRange range) {
  uint8_t gyroConfigValue = readRegister(HMS_MPU6050_GYRO_CONFIG);
  
  #ifdef HMS_MPU6050_LOGGER_ENABLED
    mpuLogger.debug("GYRO_CONFIG before: 0x%02X", gyroConfigValue);
  #endif
  
  gyroConfigValue = (gyroConfigValue & 0xE7) | ((range & 0x03) << 3);                       // Clear bits 3-4, set new range
  writeRegister(HMS_MPU6050_GYRO_CONFIG, gyroConfigValue);
  
  #ifdef HMS_MPU6050_LOGGER_ENABLED
    uint8_t verifyValue = readRegister(HMS_MPU6050_GYRO_CONFIG);
    mpuLogger.debug("GYRO_CONFIG after: 0x%02X (expected: 0x%02X)", verifyValue, gyroConfigValue);
    if (verifyValue != gyroConfigValue) {
      mpuLogger.error("Gyro range write failed!");
    }
  #endif
}

void HMS_MPU6050::setFsyncSampleOutput(HMS_MPU6050_FsyncOut fsyncOut) {
  uint8_t configValue = readRegister(HMS_MPU6050_CONFIG);
  
  #ifdef HMS_MPU6050_LOGGER_ENABLED
    mpuLogger.debug("CONFIG before: 0x%02X", configValue);
  #endif
  
  configValue = (configValue & 0x1F) | ((fsyncOut & 0x07) << 3);                            // Clear bits 3-5, set new fsync output
  writeRegister(HMS_MPU6050_CONFIG, configValue);
  
  #ifdef HMS_MPU6050_LOGGER_ENABLED
    uint8_t verifyValue = readRegister(HMS_MPU6050_CONFIG);
    mpuLogger.debug("CONFIG after: 0x%02X (expected: 0x%02X)", verifyValue, configValue);
    if (verifyValue != configValue) {
      mpuLogger.error("FSYNC sample output write failed!");
    }
  #endif
}

void HMS_MPU6050::setAccelerometerRange(HMS_MPU6050_AccelRange range) {
  uint8_t accelConfigValue = readRegister(HMS_MPU6050_ACCEL_CONFIG);
  
  #ifdef HMS_MPU6050_LOGGER_ENABLED
    mpuLogger.debug("ACCEL_CONFIG before: 0x%02X", accelConfigValue);
  #endif
  
  accelConfigValue = (accelConfigValue & 0xE7) | ((range & 0x03) << 3);                     // Clear bits 3-4, set new range
  writeRegister(HMS_MPU6050_ACCEL_CONFIG, accelConfigValue);
  
  #ifdef HMS_MPU6050_LOGGER_ENABLED
    uint8_t verifyValue = readRegister(HMS_MPU6050_ACCEL_CONFIG);
    mpuLogger.debug("ACCEL_CONFIG after: 0x%02X (expected: 0x%02X)", verifyValue, accelConfigValue);
    if (verifyValue != accelConfigValue) {
      mpuLogger.error("Accelerometer range write failed!");
    }
  #endif
}

void HMS_MPU6050::setHighPassFilter(HMS_MPU6050_HighPassFilter filter) {
  uint8_t accelConfigValue = readRegister(HMS_MPU6050_ACCEL_CONFIG);
  
  #ifdef HMS_MPU6050_LOGGER_ENABLED
    mpuLogger.debug("ACCEL_CONFIG before: 0x%02X", accelConfigValue);
  #endif
  
  accelConfigValue = (accelConfigValue & 0xF8) | (filter & 0x07);                           // Clear bits 0-2, set new filter
  writeRegister(HMS_MPU6050_ACCEL_CONFIG, accelConfigValue);
  
  #ifdef HMS_MPU6050_LOGGER_ENABLED
    uint8_t verifyValue = readRegister(HMS_MPU6050_ACCEL_CONFIG);
    mpuLogger.debug("ACCEL_CONFIG after: 0x%02X (expected: 0x%02X)", verifyValue, accelConfigValue);
    if (verifyValue != accelConfigValue) {
      mpuLogger.error("High pass filter write failed!");
    }
  #endif
}

void HMS_MPU6050::setMotionInterrupt(bool active) {
  uint8_t intEnableValue = readRegister(HMS_MPU6050_INT_ENABLE);
  
  #ifdef HMS_MPU6050_LOGGER_ENABLED
    mpuLogger.debug("INT_ENABLE before: 0x%02X", intEnableValue);
  #endif
  
  if (active) {
    intEnableValue |= 0x40;                                                                 // Set bit 6
  } else {
    intEnableValue &= 0xBF;                                                                 // Clear bit 6 (0xBF = 0b10111111)
  }
  writeRegister(HMS_MPU6050_INT_ENABLE, intEnableValue);
  
  #ifdef HMS_MPU6050_LOGGER_ENABLED
    uint8_t verifyValue = readRegister(HMS_MPU6050_INT_ENABLE);
    mpuLogger.debug("INT_ENABLE after: 0x%02X (expected: 0x%02X)", verifyValue, intEnableValue);
    if (verifyValue != intEnableValue) {
      mpuLogger.error("Motion interrupt write failed!");
    }
  #endif
}

void HMS_MPU6050::setMotionDetectionDuration(uint8_t duration) {
  #ifdef HMS_MPU6050_LOGGER_ENABLED
    uint8_t beforeValue = readRegister(HMS_MPU6050_MOT_DUR);
    mpuLogger.debug("MOT_DUR before: 0x%02X", beforeValue);
  #endif
  
  writeRegister(HMS_MPU6050_MOT_DUR, duration);
  
  // Verify the write
  #ifdef HMS_MPU6050_LOGGER_ENABLED
    uint8_t verifyValue = readRegister(HMS_MPU6050_MOT_DUR);
    mpuLogger.debug("MOT_DUR after: 0x%02X (expected: 0x%02X)", verifyValue, duration);
    if (verifyValue != duration) {
      mpuLogger.error("Motion detection duration write failed!");
    }
  #endif
}

void HMS_MPU6050::setMotionDetectionThreshold(uint8_t threshold) {
  #ifdef HMS_MPU6050_LOGGER_ENABLED
    uint8_t beforeValue = readRegister(HMS_MPU6050_MOT_THR);
    mpuLogger.debug("MOT_THR before: 0x%02X", beforeValue);
  #endif
  
  writeRegister(HMS_MPU6050_MOT_THR, threshold);
  
  // Verify the write
  #ifdef HMS_MPU6050_LOGGER_ENABLED
    uint8_t verifyValue = readRegister(HMS_MPU6050_MOT_THR);
    mpuLogger.debug("MOT_THR after: 0x%02X (expected: 0x%02X)", verifyValue, threshold);
    if (verifyValue != threshold) {
      mpuLogger.error("Motion detection threshold write failed!");
    }
  #endif
}

bool HMS_MPU6050::getMotionInterruptStatus() {
  uint8_t statusValue = readRegister(HMS_MPU6050_INT_STATUS);
  return (statusValue & 0x40) != 0;                                                        // Check bit 6
}

void HMS_MPU6050::setI2CBypass(bool bypass) {
  // Set I2C bypass bit in INT_PIN_CONFIG register (bit 1)
  uint8_t intPinConfigValue = readRegister(HMS_MPU6050_INT_PIN_CONFIG);
  if (bypass) {
    intPinConfigValue |= 0x02;                                                              // Set bit 1
  } else {
    intPinConfigValue &= 0xFD;                                                              // Clear bit 1 (0xFD = 0b11111101)
  }
  writeRegister(HMS_MPU6050_INT_PIN_CONFIG, intPinConfigValue);

  // Set I2C master enable bit in USER_CTRL register (bit 5) - opposite of bypass
  uint8_t userCtrlValue = readRegister(HMS_MPU6050_USER_CTRL);
  if (!bypass) {
    userCtrlValue |= 0x20;                                                                  // Set bit 5 (enable I2C master)
  } else {
    userCtrlValue &= 0xDF;                                                                  // Clear bit 5 (0xDF = 0b11011111)
  }
  writeRegister(HMS_MPU6050_USER_CTRL, userCtrlValue);
}

void HMS_MPU6050::setClock(HMS_MPU6050_ClockSelect clock) {
  uint8_t pwrMgmtValue = readRegister(HMS_MPU6050_PWR_MGMT_1);
  
  #ifdef HMS_MPU6050_LOGGER_ENABLED
    mpuLogger.debug("PWR_MGMT_1 before: 0x%02X", pwrMgmtValue);
  #endif
  
  pwrMgmtValue = (pwrMgmtValue & 0xF8) | (clock & 0x07);                                   // Clear bits 0-2, set new clock
  writeRegister(HMS_MPU6050_PWR_MGMT_1, pwrMgmtValue);
  
  #ifdef HMS_MPU6050_LOGGER_ENABLED
    uint8_t verifyValue = readRegister(HMS_MPU6050_PWR_MGMT_1);
    mpuLogger.debug("PWR_MGMT_1 after: 0x%02X (expected: 0x%02X)", verifyValue, pwrMgmtValue);
    if (verifyValue != pwrMgmtValue) {
      mpuLogger.error("Clock select write failed!");
    }
  #endif
}

HMS_MPU6050_ClockSelect HMS_MPU6050::getClock() {
  uint8_t pwrMgmtValue = readRegister(HMS_MPU6050_PWR_MGMT_1);
  return static_cast<HMS_MPU6050_ClockSelect>(pwrMgmtValue & 0x07);                        // Extract bits 0-2
}

bool HMS_MPU6050::enableSleep(bool enable) {
  uint8_t pwrMgmtValue = readRegister(HMS_MPU6050_PWR_MGMT_1);
  
  #ifdef HMS_MPU6050_LOGGER_ENABLED
    mpuLogger.debug("PWR_MGMT_1 before: 0x%02X", pwrMgmtValue);
  #endif
  
  if (enable) {
    pwrMgmtValue |= 0x40;                                                                   // Set bit 6
  } else {
    pwrMgmtValue &= 0xBF;                                                                   // Clear bit 6 (0xBF = 0b10111111)
  }
  writeRegister(HMS_MPU6050_PWR_MGMT_1, pwrMgmtValue);
  
  #ifdef HMS_MPU6050_LOGGER_ENABLED
    uint8_t verifyValue = readRegister(HMS_MPU6050_PWR_MGMT_1);
    mpuLogger.debug("PWR_MGMT_1 after: 0x%02X (expected: 0x%02X)", verifyValue, pwrMgmtValue);
    if (verifyValue != pwrMgmtValue) {
      mpuLogger.error("Sleep enable write failed!");
      return false;
    }
  #endif
  
  return true;
}

bool HMS_MPU6050::enableCycle(bool enable) {
  uint8_t pwrMgmtValue = readRegister(HMS_MPU6050_PWR_MGMT_1);
  
  #ifdef HMS_MPU6050_LOGGER_ENABLED
    mpuLogger.debug("PWR_MGMT_1 before: 0x%02X", pwrMgmtValue);
  #endif
  
  if (enable) {
    pwrMgmtValue |= 0x20;                                                                   // Set bit 5
  } else {
    pwrMgmtValue &= 0xDF;                                                                   // Clear bit 5 (0xDF = 0b11011111)
  }
  writeRegister(HMS_MPU6050_PWR_MGMT_1, pwrMgmtValue);
  
  #ifdef HMS_MPU6050_LOGGER_ENABLED
    uint8_t verifyValue = readRegister(HMS_MPU6050_PWR_MGMT_1);
    mpuLogger.debug("PWR_MGMT_1 after: 0x%02X (expected: 0x%02X)", verifyValue, pwrMgmtValue);
    if (verifyValue != pwrMgmtValue) {
      mpuLogger.error("Cycle enable write failed!");
      return false;
    }
  #endif
  
  return true;
}

bool HMS_MPU6050::setTemperatureStandby(bool enable) {
  uint8_t pwrMgmtValue = readRegister(HMS_MPU6050_PWR_MGMT_1);
  
  #ifdef HMS_MPU6050_LOGGER_ENABLED
    mpuLogger.debug("PWR_MGMT_1 before: 0x%02X", pwrMgmtValue);
  #endif
  
  if (enable) {
    pwrMgmtValue |= 0x08;                                                                   // Set bit 3
  } else {
    pwrMgmtValue &= 0xF7;                                                                   // Clear bit 3 (0xF7 = 0b11110111)
  }
  writeRegister(HMS_MPU6050_PWR_MGMT_1, pwrMgmtValue);
  
  #ifdef HMS_MPU6050_LOGGER_ENABLED
    uint8_t verifyValue = readRegister(HMS_MPU6050_PWR_MGMT_1);
    mpuLogger.debug("PWR_MGMT_1 after: 0x%02X (expected: 0x%02X)", verifyValue, pwrMgmtValue);
    if (verifyValue != pwrMgmtValue) {
      mpuLogger.error("Temperature standby write failed!");
      return false;
    }
  #endif
  
  return true;
}

bool HMS_MPU6050::setGyroStandby(bool xAxisStandby, bool yAxisStandby, bool zAxisStandby) {
  uint8_t pwrMgmt2Value = readRegister(HMS_MPU6050_PWR_MGMT_2);
  
  #ifdef HMS_MPU6050_LOGGER_ENABLED
    mpuLogger.debug("PWR_MGMT_2 before: 0x%02X", pwrMgmt2Value);
  #endif
  
  uint8_t gyroStandbyBits = (xAxisStandby << 2) | (yAxisStandby << 1) | zAxisStandby;      // Bits 2, 1, 0
  pwrMgmt2Value = (pwrMgmt2Value & 0xF8) | (gyroStandbyBits & 0x07);                       // Clear bits 0-2, set new values
  writeRegister(HMS_MPU6050_PWR_MGMT_2, pwrMgmt2Value);
  
  #ifdef HMS_MPU6050_LOGGER_ENABLED
    uint8_t verifyValue = readRegister(HMS_MPU6050_PWR_MGMT_2);
    mpuLogger.debug("PWR_MGMT_2 after: 0x%02X (expected: 0x%02X)", verifyValue, pwrMgmt2Value);
    if (verifyValue != pwrMgmt2Value) {
      mpuLogger.error("Gyro standby write failed!");
      return false;
    }
  #endif
  
  return true;
}

bool HMS_MPU6050::setAccelerometerStandby(bool xAxisStandby, bool yAxisStandby, bool zAxisStandby) {
  uint8_t pwrMgmt2Value = readRegister(HMS_MPU6050_PWR_MGMT_2);
  
  #ifdef HMS_MPU6050_LOGGER_ENABLED
    mpuLogger.debug("PWR_MGMT_2 before: 0x%02X", pwrMgmt2Value);
  #endif
  
  uint8_t accelStandbyBits = (xAxisStandby << 2) | (yAxisStandby << 1) | zAxisStandby;     // Bits 5, 4, 3
  pwrMgmt2Value = (pwrMgmt2Value & 0xC7) | ((accelStandbyBits & 0x07) << 3);               // Clear bits 3-5, set new values
  writeRegister(HMS_MPU6050_PWR_MGMT_2, pwrMgmt2Value);
  
  #ifdef HMS_MPU6050_LOGGER_ENABLED
    uint8_t verifyValue = readRegister(HMS_MPU6050_PWR_MGMT_2);
    mpuLogger.debug("PWR_MGMT_2 after: 0x%02X (expected: 0x%02X)", verifyValue, pwrMgmt2Value);
    if (verifyValue != pwrMgmt2Value) {
      mpuLogger.error("Accelerometer standby write failed!");
      return false;
    }
  #endif
  
  return true;
}

void HMS_MPU6050::readSensorData() {
  // Read all sensor data starting from ACCEL_OUT register (14 bytes total)
  uint8_t buffer[14];
  
  // Read 14 consecutive bytes starting from HMS_MPU6050_ACCEL_OUT
  for (int i = 0; i < 14; i++) {
    buffer[i] = readRegister(HMS_MPU6050_ACCEL_OUT + i);
  }

  // Parse accelerometer data (bytes 0-5)
  rawAccX = (buffer[0] << 8) | buffer[1];
  rawAccY = (buffer[2] << 8) | buffer[3];
  rawAccZ = (buffer[4] << 8) | buffer[5];

  // Parse temperature data (bytes 6-7)
  rawTemp = (buffer[6] << 8) | buffer[7];

  // Parse gyroscope data (bytes 8-13)
  rawGyroX = (buffer[8] << 8) | buffer[9];
  rawGyroY = (buffer[10] << 8) | buffer[11];
  rawGyroZ = (buffer[12] << 8) | buffer[13];

  // Convert temperature: (rawTemp / 340.0) + 36.53
  temperature = (rawTemp / 340.0f) + 36.53f;

  // Get accelerometer range and calculate scale
  HMS_MPU6050_AccelRange accel_range = getAccelerometerRange();
  float accel_scale = 1;
  if (accel_range == HMS_MPU6050_RANGE_16_G)
    accel_scale = 2048;
  else if (accel_range == HMS_MPU6050_RANGE_8_G)
    accel_scale = 4096;
  else if (accel_range == HMS_MPU6050_RANGE_4_G)
    accel_scale = 8192;
  else if (accel_range == HMS_MPU6050_RANGE_2_G)
    accel_scale = 16384;

  // Convert raw accelerometer data to g units
  accX = ((float)rawAccX) / accel_scale;
  accY = ((float)rawAccY) / accel_scale;
  accZ = ((float)rawAccZ) / accel_scale;

  // Get gyroscope range and calculate scale
  HMS_MPU6050_GyroRange gyro_range = getGyroRange();
  float gyro_scale = 1;
  if (gyro_range == HMS_MPU6050_RANGE_250_DEG)
    gyro_scale = 131;
  else if (gyro_range == HMS_MPU6050_RANGE_500_DEG)
    gyro_scale = 65.5f;
  else if (gyro_range == HMS_MPU6050_RANGE_1000_DEG)
    gyro_scale = 32.8f;
  else if (gyro_range == HMS_MPU6050_RANGE_2000_DEG)
    gyro_scale = 16.4f;

  // Convert raw gyroscope data to degrees/second
  gyroX = ((float)rawGyroX) / gyro_scale;
  gyroY = ((float)rawGyroY) / gyro_scale;
  gyroZ = ((float)rawGyroZ) / gyro_scale;
}

HMS_MPU6050_StatusTypeDef HMS_MPU6050::init() {
  // First, wake up the device by clearing the sleep bit
  writeRegister(HMS_MPU6050_PWR_MGMT_1, 0x00);                                            // Clear sleep bit (bit 6), use internal oscillator
  mpuDelay(100);                                                                          // Wait for device to wake up

  if(readRegister(HMS_MPU6050_WHO_AM_I) != HMS_MPU6050_DEVICE_ID) {
    #ifdef HMS_MPU6050_LOGGER_ENABLED
      mpuLogger.error(
        "Device ID mismatch. Expected 0x%02X, got 0x%02X", 
        HMS_MPU6050_DEVICE_ID, readRegister(HMS_MPU6050_WHO_AM_I)
      );
    #endif
    return HMS_MPU6050_ERROR;                                                             // Device ID mismatch
  }

  reset();
  
  // Wake up again after reset since reset puts device back to sleep
  writeRegister(HMS_MPU6050_PWR_MGMT_1, 0x00);                                            // Clear sleep bit again
  mpuDelay(100);
  
  setSampleRateDivisor(4);                                                                // Set sample rate to 200Hz (
  setFilterBandwidth(HMS_MPU6050_BAND_21_HZ);                                             // Set DLPF to 21Hz
  setGyroRange(HMS_MPU6050_RANGE_250_DEG);                                                // Set gyro range to +/- 250 deg/s
  setAccelerometerRange(HMS_MPU6050_RANGE_2_G);                                           // Set accelerometer range to +/- 2g

  writeRegister(HMS_MPU6050_PWR_MGMT_1, 0x01);                                            // Set clock config to PLL with Gyro X reference
  mpuDelay(100);

  #ifdef HMS_MPU6050_LOGGER_ENABLED
    mpuLogger.info("MPU6050 initialization completed successfully");
  #endif
  return HMS_MPU6050_OK;
}

uint8_t HMS_MPU6050::getSampleRateDivisor() {
    return readRegister(HMS_MPU6050_SMPLRT_DIV);
}

HMS_MPU6050_GyroRange HMS_MPU6050::getGyroRange() {
  uint8_t gyroConfigValue = readRegister(HMS_MPU6050_GYRO_CONFIG);
  return static_cast<HMS_MPU6050_GyroRange>((gyroConfigValue >> 3) & 0x03);
}

HMS_MPU6050_Bandwidth HMS_MPU6050::getFilterBandwidth() {
  uint8_t configValue = readRegister(HMS_MPU6050_CONFIG);
  return static_cast<HMS_MPU6050_Bandwidth>(configValue & 0x07);
}

HMS_MPU6050_AccelRange HMS_MPU6050::getAccelerometerRange() {
  uint8_t accelConfigValue = readRegister(HMS_MPU6050_ACCEL_CONFIG);
  return static_cast<HMS_MPU6050_AccelRange>((accelConfigValue >> 3) & 0x03);
}

HMS_MPU6050_HighPassFilter HMS_MPU6050::getHighPassFilter() {
  uint8_t configValue = readRegister(HMS_MPU6050_ACCEL_CONFIG);
  return static_cast<HMS_MPU6050_HighPassFilter>(configValue & 0x07);
}

void HMS_MPU6050::getSensorData(HMS_MPU6050_SensorData* sensor) {
  readSensorData();
  if (sensor) {
    sensor->Accel.x = accX;
    sensor->Accel.y = accY;
    sensor->Accel.z = accZ;
    sensor->Gyro.x = gyroX;
    sensor->Gyro.y = gyroY;
    sensor->Gyro.z = gyroZ;
    sensor->Temperature = temperature;
  }
}

void HMS_MPU6050::getAcceleration(HMS_MPU6050_Acceleration* accel) {
  readSensorData();
  if (accel) {
    accel->x = accX;
    accel->y = accY;
    accel->z = accZ;
  }
}

void HMS_MPU6050::getGyroscope(HMS_MPU6050_Gyroscope* gyro) {
  readSensorData();
  if (gyro) {
    gyro->x = gyroX;
    gyro->y = gyroY;
    gyro->z = gyroZ;
  }
}