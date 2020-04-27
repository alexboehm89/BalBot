/**
 * @file gyroscope_L3GD20.h
 * @author Alexander Boehm (alex.boehm89@gmail.com)
 * @brief  A class to interface with the L3GD20 gyroscope
 * @version 0.1
 * @date 2020-04-12
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#ifndef GYROSCOPE_L3GD20_H
#define GYROSCOPE_L3GD20_H

#include <cstdint>
#include "adafruit_9dof_imu/imu_sensor.h"

/**
 * @brief  Define I2C address and gyroscope ID
 * 
 */
#define GYRO_ADDRESS      (0x69)
#define GYRO_ID           (0xD3)


/**
 * @brief  Define all registers
 * 
 */
typedef enum
{                              // DEFAULT    Type
  GYRO_WHO_AM_I      = 0x0F,   // 11010100   r
  GYRO_CTRL_REG1     = 0x20,   // 00000111   rw
  GYRO_CTRL_REG2     = 0x21,   // 00000000   rw
  GYRO_CTRL_REG3     = 0x22,   // 00000000   rw
  GYRO_CTRL_REG4     = 0x23,   // 00000000   rw
  GYRO_CTRL_REG5     = 0x24,   // 00000000   rw
  GYRO_REFERENCE     = 0x25,   // 00000000   rw
  GYRO_OUT_TEMP      = 0x26,   //            r
  GYRO_STATUS_REG    = 0x27,   //            r
  GYRO_OUT_X_L       = 0x28,   //            r
  GYRO_OUT_X_H       = 0x29,   //            r
  GYRO_OUT_Y_L       = 0x2A,   //            r
  GYRO_OUT_Y_H       = 0x2B,   //            r
  GYRO_OUT_Z_L       = 0x2C,   //            r
  GYRO_OUT_Z_H       = 0x2D,   //            r
  GYRO_FIFO_CTRL_REG = 0x2E,   // 00000000   rw
  GYRO_FIFO_SRC_REG  = 0x2F,   //            r
  GYRO_INT1_CFG      = 0x30,   // 00000000   rw
  GYRO_INT1_SRC      = 0x31,   //            r
  GYRO_TSH_XH        = 0x32,   // 00000000   rw
  GYRO_TSH_XL        = 0x33,   // 00000000   rw
  GYRO_TSH_YH        = 0x34,   // 00000000   rw
  GYRO_TSH_YL        = 0x35,   // 00000000   rw
  GYRO_TSH_ZH        = 0x36,   // 00000000   rw
  GYRO_TSH_ZL        = 0x37,   // 00000000   rw
  GYRO_INT1_DURATION = 0x38    // 00000000   rw
} gyroRegisters_t;


/**
 * @brief  Define input parameters of gyroscope
 * 
 */
typedef struct
{
  int rate;
  int range;
  bool isAutoRange;
} gyroParams_t;


/**
 * @brief  Define class to interface L3GD20 gyroscope.
 *         This class inherits interfaces from IMUSensor class.
 * 
 */
class GyroscopeL3GD20 : public IMUSensor
{
public:
  /**
   * @brief  Constructor  
   * 
   */
  GyroscopeL3GD20();

  /**
   * @brief  Destructor
   * 
   */
  ~GyroscopeL3GD20();

  /**
   * @brief  Setup of gyroscope
   * 
   * @param  params      Object that holds default parametes
   * @return bool        True if setup was successful, false otherwise
   */
   bool setup(gyroParams_t params);

  /**
   * @brief  Set all registers to default value and enable normal mode
   * 
   * @return bool        True if reset was successful, false otherwise
   */
  bool reset();

  /**
   * @brief  Powers down gyroscope. The power consumption is ca. 5µA.
   * 
   * @return bool        True if gyroscope is powered down, false otherwise
   */
  bool powerDown();

  /**
   * @brief  Sends gyroscope to sleep. The power consumption is 2mA.
   * 
   * @return bool        True if gyroscope is asleep, false otherwise
   */
  bool sleep();

  /**
   * @brief  Enables normal mode of gyroscope. The power consumption is 6.1 mA.
   * 
   * @return bool        True if gyroscope is in normal mode, false otherwise
   */
  bool setNormalMode();

  /**
   * @brief  Function to read data from gyroscope
   * 
   * @param  data        Object to save gyroscope data
   * @return bool        True if data was saved, false otherwise
   */
  bool getData(sensorData_t &data);

  /**
   * @brief  Function to set output data rate of gyroscope
   * 
   * @param  rate        Desired output data rate of gyroscope
   * @return bool        True if output data rate was set, false otherwise
   */
  bool setRate(int rate);

  /**
   * @brief  Function to set output data range of gyroscope
   * 
   * @param  range       Desired output data range of gyroscope
   * @return bool        True if output data range was set, false otherwise
   */
  bool setRange(int range);

private:
  /**
   * @brief  Reboots memory content of gyroscope
   * 
   * @return bool        True if successful, false otherwise
   */
  bool rebootMemory();

  /**
   * @brief  Function to change single bits of a value
   * 
   * @param  action      Integer that defines what action should be taken
   *                     0: Set bit
   *                     1: Clear bit
   *                     2: Toggle bit
   * @param  bit         This bit will be changed. An input 0 refers to LSB
   * @param  val         A bit from this value will be changed
   * @return bool        True if action was successful, false otherwise
   */
  bool handleBit(int action, int bit, uint8_t &val);


  // Current and default output data range
  int range_;
  int def_range_;

  // Current and default output data rate
  int rate_;
  int def_rate_;

  // Sensitivy values of gyroscope per datasheet
  const double SENSITIVITY_250DPS_  = 0.00875;
  const double SENSITIVITY_500DPS_  = 0.0175;
  const double SENSITIVITY_2000DPS_ = 0.07;

  // Current sensitivity (based on range)
  double sensitivity_ = 0;

  // Factor to transform degree to rad
  const double DEG2RAD_ = 3.14159265359f / 180;

  // 
  bool isAutoRange_ = false;

  // File descriptor for I2C interface
  int fd_ = -1;
};


#endif
