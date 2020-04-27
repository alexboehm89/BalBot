/**
 * @file gyroscope_L3GD20.cpp
 * @author Alexander Boehm (alex.boehm89@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2020-04-12
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#include "adafruit_9dof_imu/gyroscope_L3GD20.h"
#include <wiringPiI2C.h>

GyroscopeL3GD20::GyroscopeL3GD20()
{
}


GyroscopeL3GD20::~GyroscopeL3GD20()
{
}


bool GyroscopeL3GD20::setup(gyroParams_t params)
{
  // Setup I2C communication
  fd_ = wiringPiI2CSetup(GYRO_ADDRESS);

  // Get sensor ID
  uint8_t sensor_id = wiringPiI2CReadReg8(fd_, GYRO_WHO_AM_I);

  // Check if setup is correct
  if (GYRO_ID != sensor_id)
  {
    return false;
  }

  // Read parameters and set default/current values
  def_range_   = params.range;
  def_rate_    = params.rate;
  range_       = def_range_;
  rate_        = def_rate_;
  isAutoRange_ = params.isAutoRange;

  // Reset gyroscope
  reset();

  return true;
}


bool GyroscopeL3GD20::reset()
{
  // Set all registers to their default value per datasheet
  wiringPiI2CWriteReg8(fd_, GYRO_CTRL_REG1,     0x07);
  wiringPiI2CWriteReg8(fd_, GYRO_CTRL_REG2,     0x00);
  wiringPiI2CWriteReg8(fd_, GYRO_CTRL_REG3,     0x00);
  wiringPiI2CWriteReg8(fd_, GYRO_CTRL_REG4,     0x00); 
  wiringPiI2CWriteReg8(fd_, GYRO_CTRL_REG5,     0x00);
  wiringPiI2CWriteReg8(fd_, GYRO_REFERENCE,     0x00);
  wiringPiI2CWriteReg8(fd_, GYRO_FIFO_CTRL_REG, 0x00);
  wiringPiI2CWriteReg8(fd_, GYRO_INT1_CFG,      0x00);
  wiringPiI2CWriteReg8(fd_, GYRO_TSH_XH,        0x00);
  wiringPiI2CWriteReg8(fd_, GYRO_TSH_XL,        0x00);
  wiringPiI2CWriteReg8(fd_, GYRO_TSH_YH,        0x00);
  wiringPiI2CWriteReg8(fd_, GYRO_TSH_YL,        0x00);
  wiringPiI2CWriteReg8(fd_, GYRO_TSH_ZH,        0x00);
  wiringPiI2CWriteReg8(fd_, GYRO_TSH_ZL,        0x00);
  wiringPiI2CWriteReg8(fd_, GYRO_INT1_DURATION, 0x00);

  // Enable Big Endian for faster data access 
  //wiringPiI2CWriteReg8(fd_, GYRO_CTRL_REG4, 0x40);

  // Enable all axis and normal operating mode
  wiringPiI2CWriteReg8(fd_, GYRO_CTRL_REG1, 0x0F);

  // Set up gyroscope according to default parameters
  setRange(def_range_);
  setRate(def_rate_);

  // Reboot memory content
  //rebootMemory();

  return true;
}

bool GyroscopeL3GD20::powerDown()
{
  // Read current value of CTRL_REG1
  uint8_t val = wiringPiI2CReadReg8(fd_, GYRO_CTRL_REG1);
  
  // Adjust value as needed and write to CTRL_REG1
  // CTRL_REG1: xxxx 0xxx
  handleBit(2, 3, val);
  wiringPiI2CWriteReg8(fd_,GYRO_CTRL_REG1, val);

  return true;
}


bool GyroscopeL3GD20::sleep()
{
  // Read current value of CTRL_REG1
  uint8_t val = wiringPiI2CReadReg8(fd_, GYRO_CTRL_REG1);
  
  // Adjust value as needed and write to CTRL_REG1
  // CTRL_REG1: xxxx 1000
  handleBit(1, 3, val);
  handleBit(2, 2, val);
  handleBit(2, 1, val);
  handleBit(2, 0, val);
  wiringPiI2CWriteReg8(fd_, GYRO_CTRL_REG1, val);

  return true;
}


bool GyroscopeL3GD20::setNormalMode()
{
  // Read current value of CTRL_REG1
  uint8_t val = wiringPiI2CReadReg8(fd_, GYRO_CTRL_REG1);
  
  // Adjust value as needed and write to CTRL_REG1
  // CTRL_REG1: xxxx 1111
  handleBit(1, 3, val);
  handleBit(1, 2, val);
  handleBit(1, 1, val);
  handleBit(1, 0, val);
  wiringPiI2CWriteReg8(fd_, GYRO_CTRL_REG1, val);

  // Reboot memory
  rebootMemory();

  return true;
}


bool GyroscopeL3GD20::rebootMemory()
{
  // Read current value of CTRL_REG5
  uint8_t val = wiringPiI2CReadReg8(fd_, GYRO_CTRL_REG5);

  // Adjust value to trigger memory reboot and write to CTRL_REG5
  handleBit(1, 7, val);
  wiringPiI2CWriteReg8(fd_, GYRO_CTRL_REG5, val);

  // Reset value
  handleBit(2, 7, val);
  wiringPiI2CWriteReg8(fd_, GYRO_CTRL_REG5, val);

  return true;
}


bool GyroscopeL3GD20::getData(sensorData_t &data)
{
  // Initialize variables for measurements
  uint8_t xVal_L;
  uint8_t xVal_H;
  uint8_t yVal_L;
  uint8_t yVal_H;
  uint8_t zVal_L;
  uint8_t zVal_H;
  int16_t xVal;
  int16_t yVal;
  int16_t zVal;

  // Checkif new data is available
  //uint8_t val = wiringPiI2CReadReg8(fd_, GYRO_STATUS_REG);
  //bool isDataAvail = (val >> 3) & 1U;
  // Read data registers. Caution! Big endian has to be enabled! 
  xVal_L = wiringPiI2CReadReg8(fd_, GYRO_OUT_X_L);
  xVal_H = wiringPiI2CReadReg8(fd_, GYRO_OUT_X_H);
  yVal_L = wiringPiI2CReadReg8(fd_, GYRO_OUT_Y_L);
  yVal_H = wiringPiI2CReadReg8(fd_, GYRO_OUT_Y_H);
  zVal_L = wiringPiI2CReadReg8(fd_, GYRO_OUT_Z_L);
  zVal_H = wiringPiI2CReadReg8(fd_, GYRO_OUT_Z_H);
  xVal = (int16_t) (xVal_L | (xVal_H << 8));
  yVal = (int16_t) (yVal_L | (yVal_H << 8));
  zVal = (int16_t) (zVal_L | (zVal_H << 8));
  // Compensate values depending on range
  data.x = xVal*sensitivity_*DEG2RAD_;
  data.y = yVal*sensitivity_*DEG2RAD_;
  data.z = zVal*sensitivity_*DEG2RAD_;

  // Check if auto-ranging is active
  if (isAutoRange_)
  {
    // Check if any measurement is saturating
    if ((xVal < -28571) || (xVal > 28571) ||
        (yVal < -28571) || (yVal > 28571) ||
        (zVal < -28571) || (zVal > 28571))
    {
      // Adjust range
      switch (range_)
      {
        case 250:
          setRange(500);
          break;
        case 500:
          setRange(2000);
          break;
        case 2000:
          // Already greatest range, skip loop
          break;
      }
    }
    else
    {
      switch (range_)
      {
        case 2000:
          if ((xVal < 7142) && (xVal > -7142) &&
              (yVal < 7142) && (yVal > -7142) &&
              (zVal < 7142) && (zVal > -7142))
          {
            setRange(500);
          }
          break;
        case 500:
          if ((xVal < 14704) && (xVal > -14704) &&
              (yVal < 14704) && (yVal > -14704) &&
              (zVal < 14704) && (zVal > -14704))
          {
            setRange(250);
          }
          break;
      }
    }
  }

  return true;
}


bool GyroscopeL3GD20::setRate(int rate)
{
  // Save requested rate
  rate_ = rate;

  // Read current value of CTRL_REG1
  uint8_t val = wiringPiI2CReadReg8(fd_, GYRO_CTRL_REG1);

  // Adjust value as needed 
  switch (rate_)
  {
    case 95:
      // CTRL_REG1: 00xx xxxx
      handleBit(2, 7, val);
      handleBit(2, 6, val);
      break;
    case 190:
      // CTRL_REG1: 01xx xxxx
      handleBit(2, 7, val);
      handleBit(1, 6, val);
      break;
    case 380:
      // CTRL_REG1: 10xx xxxx
      handleBit(1, 7, val);
      handleBit(2, 6, val);
      break;
    case 760:
      // CTRL_REG1: 11xx xxxx
      handleBit(1, 7, val);
      handleBit(1, 6, val);
      break;
    default:
      // No valid rate was selected
      return false;
  }

  // Write value to CTRL_REG1
  wiringPiI2CWriteReg8(fd_, GYRO_CTRL_REG1, val);

  return true;
}


bool GyroscopeL3GD20::setRange(int range)
{
  // Save requested range
  range_ = range;
  
  // Read current value of CTRL_REG4
  uint8_t val = wiringPiI2CReadReg8(fd_, GYRO_CTRL_REG4);

  // Adjust value as needed 
  switch (range_)
  {
    case 250:
      // CTRL_REG4: xx00 xxxx
      handleBit(2, 5, val);
      handleBit(2, 4, val);
      // Set sensitivity value
      sensitivity_ = SENSITIVITY_250DPS_;
      break;
    case 500:
      // CTRL_REG4: xx01 xxxx
      handleBit(2, 5, val);
      handleBit(1, 4, val);
      // Set sensitivity value
      sensitivity_ = SENSITIVITY_500DPS_;
      break;
    case 2000:
      // CTRL_REG4: xx10 xxxx
      handleBit(1, 5, val);
      handleBit(2, 4, val);
      // Set sensitivity value
      sensitivity_ = SENSITIVITY_2000DPS_;
      break;
    default:
      // No valid range was selected
      return false;
  }

  // Write value to CTRL_REG4
  wiringPiI2CWriteReg8(fd_, GYRO_CTRL_REG4, val);

  return true;
}


bool GyroscopeL3GD20::handleBit(int action, int bit, uint8_t &val)
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
