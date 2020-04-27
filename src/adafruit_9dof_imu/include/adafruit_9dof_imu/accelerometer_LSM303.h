/**
 * @file accelerometer_LSM303.h
 * @author Alexander Boehm (alex.boehm89@gmail.com)
 * @brief  A class to interface with the LSM303 accelerometer
 * @version 0.1
 * @date 2020-04-18
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#ifndef ACCELEROMETER_LSM303_H
#define ACCELEROMETER_LSM303_H

#include <cstdint>
#include "adafruit_9dof_imu/imu_sensor.h"


/**
 * @brief  Define I2C address
 * 
 */
#define ACCEL_ADDRESS (0x19)


/**
 * @brief  Define all registers
 * 
 */
typedef enum
{                               // DEFAULT    TYPE
  ACCEL_CTRL_REG1     = 0x20,   // 00000111   rw
  ACCEL_CTRL_REG2     = 0x21,   // 00000000   rw
  ACCEL_CTRL_REG3     = 0x22,   // 00000000   rw
  ACCEL_CTRL_REG4     = 0x23,   // 00000000   rw
  ACCEL_CTRL_REG5     = 0x24,   // 00000000   rw
  ACCEL_CTRL_REG6     = 0x25,   // 00000000   rw
  ACCEL_REFERENCE     = 0x26,   // 00000000   r
  ACCEL_STATUS_REG    = 0x27,   // 00000000   r
  ACCEL_OUT_X_L       = 0x28,   //            r
  ACCEL_OUT_X_H       = 0x29,   //            r
  ACCEL_OUT_Y_L       = 0x2A,   //            r
  ACCEL_OUT_Y_H       = 0x2B,   //            r
  ACCEL_OUT_Z_L       = 0x2C,   //            r
  ACCEL_OUT_Z_H       = 0x2D,   //            r
  ACCEL_FIFO_CTRL_REG = 0x2E,   // 00000000   rw
  ACCEL_FIFO_SRC_REG  = 0x2F,   //            r
  ACCEL_INT1_CFG      = 0x30,   // 00000000   rw
  ACCEL_INT1_SOURCE   = 0x31,   // 00000000   r
  ACCEL_INT1_THS      = 0x32,   // 00000000   rw
  ACCEL_INT1_DURATION = 0x33,   // 00000000   rw
  ACCEL_INT2_CFG      = 0x34,   // 00000000   rw
  ACCEL_INT2_SOURCE   = 0x35,   // 00000000   r
  ACCEL_INT2_THS      = 0x36,   // 00000000   rw
  ACCEL_INT2_DURATION = 0x37,   // 00000000   rw
  ACCEL_CLICK_CFG     = 0x38,   // 00000000   rw
  ACCEL_CLICK_SRC     = 0x39,   // 00000000   rw
  ACCEL_CLICK_THS     = 0x3A,   // 00000000   rw
  ACCEL_TIME_LIMIT    = 0x3B,   // 00000000   rw
  ACCEL_TIME_LATENCY  = 0x3C,   // 00000000   rw
  ACCEL_TIME_WINDOW   = 0x3D    // 00000000   rw
} accelRegisters_t;


/**
 * @brief  Define input parameters if accelerometer
 * 
 */
typedef struct
{
  int rate;
  int range;
} accelParams_t;


/**
 * @brief  Define class to interface LSM303 accelerometer.
 *         This class inherits interfaces from IMUSensor class.
 * 
 */
class AccelerometerLSM303 : public IMUSensor
{
public:
  /**
   * @brief  Constructor
   * 
   */
  AccelerometerLSM303();

  /**
   * @brief Destructor
   * 
   */
  ~AccelerometerLSM303();

  /**
   * @brief  Setup of accelerometer
   * 
   * @param  params      Object that holds default parameters
   * @return bool        True if setup was successful, false otherwise
   */
  bool setup(accelParams_t params);

  /**
   * @brief  Set all registers to default value and enable normal mode
   * 
   * @return bool        True if reset was successful, false otherwise
   */
  bool reset();

  /**
   * @brief  Powers down accelerometer.
   * 
   * @return bool        True if accelerometer is powered down, false otherwise
   */
  bool powerDown();

  /**
   * @brief  Enables low power mode
   * 
   * @return bool        True if low power mode is enabled, false otherwise
   */
  bool lowPowerMode();

  /**
   * @brief  Enables normal mode of accelerometer
   * 
   * @return bool        True if accelerometer is in normal mode, false otherwise
   */
  bool setNormalMode();

  /**
   * @brief  Function to read data from accelerometer
   * 
   * @param  data        Object to save accelerometer data
   * @return bool        True if data was saved, false otherwise
   */
  bool getData(sensorData_t &data);

  /**
   * @brief  Function to set output data rate of accelerometer
   * 
   * @param  rate        Desired output data rate of accelerometer
   * @return bool        True if output data rate was set, false otherwise
   */
  bool setRate(int rate);

  /**
   * @brief  Function to set output data range of accelerometer
   * 
   * @param  range       Desired output data range of accelerometer
   * @return bool        True if output data range was set, false otherwise
   */
  bool setRange(int range);

private:
  /**
   * @brief  Reboots memory content of accelerometer
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


  // Sensitivity values of accelerometer per datasheet
  const double SENSITIVITY_2G_  = 0.001;
  const double SENSITIVITY_4G_  = 0.002;
  const double SENSITIVITY_8G_  = 0.004;
  const double SENSITIVITY_16G_ = 0.012;

  // Current sensitivity (based on range)
  double sensitivity_ = 0;

  // Factor to tanform g to m/s² (in Bavaria)
  const double G2MPS2_ = 9.807f; 

  // File desctiptor for I2C interface
  int fd_ = -1;
};

#endif
