/**
 * @file accelerometer_LSM303.cpp
 * @author Alexander Boehm (alex.boehm89@gmail.com)
 * @brief  
 * @version 0.1
 * @date 2020-04-18
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#include "adafruit_9dof_imu/accelerometer_LSM303.h"
#include <wiringPiI2C.h>

AccelerometerLSM303::AccelerometerLSM303()
{
}

AccelerometerLSM303::~AccelerometerLSM303()
{
}


bool AccelerometerLSM303::setup(accelParams_t params)
{
  // Setup I2C communication
  fd_ = wiringPiI2CSetup(ACCEL_ADDRESS);
  
  // Read parameters and set default/current values
  def_range_ = params.range;
  def_rate_  = params.rate;
  range_     = def_range_;
  rate_      = def_rate_;

  // Reset accelerometer
  reset();

  return true;
}


bool AccelerometerLSM303::reset()
{
  // Set all registers to their default value
  wiringPiI2CWriteReg8(fd_, ACCEL_CTRL_REG1,     0x07);
  wiringPiI2CWriteReg8(fd_, ACCEL_CTRL_REG2,     0x00);
  wiringPiI2CWriteReg8(fd_, ACCEL_CTRL_REG3,     0x00);
  wiringPiI2CWriteReg8(fd_, ACCEL_CTRL_REG4,     0x00);
  wiringPiI2CWriteReg8(fd_, ACCEL_CTRL_REG5,     0x00);
  wiringPiI2CWriteReg8(fd_, ACCEL_CTRL_REG6,     0x00);
  wiringPiI2CWriteReg8(fd_, ACCEL_FIFO_CTRL_REG, 0x00);
  wiringPiI2CWriteReg8(fd_, ACCEL_INT1_CFG,      0x00);
  wiringPiI2CWriteReg8(fd_, ACCEL_INT1_THS,      0x00);
  wiringPiI2CWriteReg8(fd_, ACCEL_INT1_DURATION, 0x00);
  wiringPiI2CWriteReg8(fd_, ACCEL_INT2_CFG,      0x00);
  wiringPiI2CWriteReg8(fd_, ACCEL_INT2_THS,      0x00);
  wiringPiI2CWriteReg8(fd_, ACCEL_INT2_DURATION, 0x00);
  wiringPiI2CWriteReg8(fd_, ACCEL_CLICK_CFG,     0x00);
  wiringPiI2CWriteReg8(fd_, ACCEL_CLICK_SRC,     0x00);
  wiringPiI2CWriteReg8(fd_, ACCEL_CLICK_THS,     0x00);
  wiringPiI2CWriteReg8(fd_, ACCEL_TIME_LIMIT,    0x00);
  wiringPiI2CWriteReg8(fd_, ACCEL_TIME_LATENCY,  0x00);
  wiringPiI2CWriteReg8(fd_, ACCEL_TIME_WINDOW,   0x00);

  // Enable Big Endian for faster data access (does not seem to work)
  wiringPiI2CWriteReg8(fd_, ACCEL_CTRL_REG4, 0x80);

  // Set up accelerometer according to default parameters
  setRange(def_range_);
  setRate(def_rate_);

  // Reboot memory content
  rebootMemory();

  return true;
}


bool AccelerometerLSM303::powerDown()
{
  // Read current value of CTRL_REG1
  uint8_t val = wiringPiI2CReadReg8(fd_, ACCEL_CTRL_REG1);

  // Adjust value as needed and write to CTRL_REG1
  // CTRL_REG1: 0000 xxxx
  handleBit(2, 7, val);
  handleBit(2, 6, val);
  handleBit(2, 5, val);
  handleBit(2, 4, val);
  wiringPiI2CWriteReg8(fd_, ACCEL_CTRL_REG1, val);

  return true;
}

bool AccelerometerLSM303::lowPowerMode()
{
  // Read current value of CTRL_REG1
  uint8_t val = wiringPiI2CReadReg8(fd_, ACCEL_CTRL_REG1);

  // Adjust value as needed and write to CTRL_REG1
  // CTRL_REG1: xxxx 1xxx
  handleBit(1, 3, val);
  wiringPiI2CWriteReg8(fd_, ACCEL_CTRL_REG1, val);
  // Set rate to disable power down mode
  setRate(rate_);

  return true;
}

bool AccelerometerLSM303::setNormalMode()
{
  // Read current value of CTRL_REG1
  uint8_t val = wiringPiI2CReadReg8(fd_, ACCEL_CTRL_REG1);

  // Adjust value as needed and write to CTRL_REG1
  // CTRL_REG1: xxxx 0xxxx
  handleBit(2, 3, val);
  wiringPiI2CWriteReg8(fd_, ACCEL_CTRL_REG1, val);

  // Set rate to disable power down mode
  setRate(rate_);

  return true;
}

bool AccelerometerLSM303::rebootMemory()
{
  // Read current value of ACCEL_CTRL_REG5
  uint8_t val = wiringPiI2CReadReg8(fd_, ACCEL_CTRL_REG5);

  // Adjust value to trigger memory reboot and write to CTRL_REG5
  handleBit(1, 7, val);
  wiringPiI2CWriteReg8(fd_, ACCEL_CTRL_REG5, val);

  // Reset value
  handleBit(2, 7, val);
  wiringPiI2CWriteReg8(fd_, ACCEL_CTRL_REG5, val);

  return true;
}


bool AccelerometerLSM303::getData(sensorData_t &data)
{
  // Read data registers. Caution! Little endian has to be enabled!
  uint8_t xVal_L = wiringPiI2CReadReg8(fd_, ACCEL_OUT_X_L);
  uint8_t xVal_H = wiringPiI2CReadReg8(fd_, ACCEL_OUT_X_H);
  uint8_t yVal_L = wiringPiI2CReadReg8(fd_, ACCEL_OUT_Y_L);
  uint8_t yVal_H = wiringPiI2CReadReg8(fd_, ACCEL_OUT_Y_H);
  uint8_t zVal_L = wiringPiI2CReadReg8(fd_, ACCEL_OUT_Z_L);
  uint8_t zVal_H = wiringPiI2CReadReg8(fd_, ACCEL_OUT_Z_H);

  // Shift values to create proper data set (low byte first)
  // Only 12 bits left-assigned are used
  int16_t xVal = (int16_t)(xVal_L | (xVal_H << 8)) >> 4;
  int16_t yVal = (int16_t)(yVal_L | (yVal_H << 8)) >> 4;
  int16_t zVal = (int16_t)(zVal_L | (zVal_H << 8)) >> 4;  

  // Compensate values depending on range
  data.x = xVal*sensitivity_*G2MPS2_;
  data.y = yVal*sensitivity_*G2MPS2_;
  data.z = zVal*sensitivity_*G2MPS2_;

  return true;
}


bool AccelerometerLSM303::setRate(int rate)
{
  // Save requested rate
  rate_ = rate;

  // Read current value of ACCEL_CTRL_REG1
  uint8_t val = wiringPiI2CReadReg8(fd_, ACCEL_CTRL_REG1);

  // Adjust value as needed
  switch (rate_)
  {
    case 1:
      // CTRL_REG1_A: 0001 xxxx
      handleBit(2, 7, val);
      handleBit(2, 6, val);
      handleBit(2, 5, val);
      handleBit(1, 4, val);
      break;
    case 10:
      // CTRL_REG1_A: 0010 xxxx
      handleBit(2, 7, val);
      handleBit(2, 6, val);
      handleBit(1, 5, val);
      handleBit(2, 4, val);
      break;
    case 25:
      // CTRL_REG1_A: 0011 xxxx
      handleBit(2, 7, val);
      handleBit(2, 6, val);
      handleBit(1, 5, val);
      handleBit(1, 4, val);
      break;
    case 50:
      // CTRL_REG1_A: 0100 xxxx
      handleBit(2, 7, val);
      handleBit(1, 6, val);
      handleBit(2, 5, val);
      handleBit(2, 4, val);
      break;
    case 100:
      // CTRL_REG1_A: 0101 xxxx
      handleBit(2, 7, val);
      handleBit(1, 6, val);
      handleBit(2, 5, val);
      handleBit(1, 4, val);
      break;
    case 200:
      // CTRL_REG1_A: 0110 xxxx
      handleBit(2, 7, val);
      handleBit(1, 6, val);
      handleBit(1, 5, val);
      handleBit(2, 4, val);
      break;
    case 400:
      // CTRL_REG1_A: 0111 xxxx
      handleBit(2, 7, val);
      handleBit(1, 6, val);
      handleBit(1, 5, val);
      handleBit(1, 4, val);
      break;
    default:
      // No valid rate was selected
      return false;
  }

  // write valueto ACCEL_CTRL_REG1
  wiringPiI2CWriteReg8(fd_, ACCEL_CTRL_REG1, val);

  return true;
}


bool  AccelerometerLSM303::setRange(int range)
{
  // Save requested range
  range_ = range;

  // Read current value of ACCEL_CTRL_REG4
  uint8_t val = wiringPiI2CReadReg8(fd_, ACCEL_CTRL_REG4);

  // Adjust value as needed
  switch (range_)
  {
    case 2:
      // CTRL_REG4_A: xx00 xxxx
      handleBit(2, 5, val);
      handleBit(2, 4, val);
      // Set sensitivitiy value
      sensitivity_ = SENSITIVITY_2G_;
      break;
    case 4:
      // CTRL_REG4_A: xx01 xxxx
      handleBit(2, 5, val);
      handleBit(1, 4, val);
      // Set sensitivitiy value
      sensitivity_ = SENSITIVITY_4G_;
      break;
    case 8:
      // CTRL_REG4_A: xx10 xxxx
      handleBit(1, 5, val);
      handleBit(2, 4, val);
      // Set sensitivitiy value
      sensitivity_ = SENSITIVITY_8G_;
      break;
    case 16:
      // CTRL_REG4_A: xx11 xxxx
      handleBit(1, 5, val);
      handleBit(1, 4, val);
      // Set sensitivitiy value
      sensitivity_ = SENSITIVITY_16G_;
      break;
    default:
      // No valid range was selected
      return false;
  }

  // Write value to ACCEL_CTRL_REG4
  wiringPiI2CWriteReg8(fd_, ACCEL_CTRL_REG4, val);

  return true;
}


bool AccelerometerLSM303::handleBit(int action, int bit, uint8_t &val)
{
  switch (action)
  {
    // Set bit
    case 1:
      val |= 1UL << bit;
      break;
    // Clear bit
    case 2:
      val &= ~(1UL << bit);
      break;
    // Toggle bit
    case 3:
      val ^= 1UL << bit;
      break;
    default:
      return false;
  }

  return true;
}
