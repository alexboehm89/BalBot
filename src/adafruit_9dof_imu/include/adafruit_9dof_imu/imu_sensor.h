/**
 * @file imu_sensor.h
 * @author Alexander Boehm (alex.boehm89@gmail.com)
 * @brief  Base class for IMU sensors
 * @version 0.1
 * @date 2020-04-18
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#ifndef IMU_SENSOR_H
#define IMU_SENSOR_H

#include <cstdint>

/**
 * @brief  Object to store sensor data
 * 
 */
typedef struct
{
  double x;
  double y;
  double z;
} sensorData_t;

class IMUSensor 
{
public:
  /**
   * @brief  Pure virtual function to reset sensor
   * 
   * @return bool        True if reset was successful, false otherwise
   */
  virtual bool reset() = 0;

  /**
   * @brief  Pure virtual funtion to power down sensor
   * 
   * @return bool        True if sensor is powered down, false otherwise
   */
  virtual bool powerDown() = 0;

  /**
   * @brief  Pure virtual function to enable normal mode
   * 
   * @return bool        True if sensor is in normal mode, false otherwise
   */
  virtual bool setNormalMode() = 0;

protected:
  /**
   * @brief  Pure virtual function to reboot memory content of sensor
   * 
   * @return bool        True if reboot was successful, false otherwise
   */
  virtual bool rebootMemory() = 0;


  /**
   * @brief  Pure virtual function to change single bits of a value
   * 
   * @param  action      Integer that defines what action should be taken
   *                     0: Set bit
   *                     1: Clear bit
   *                     2: Toggle bit
   * @param  bit         This bit will be changed. An input 0 refers to LSB
   * @param  val         A bit from this value will be changed
   * @return bool        True if action was successful, false otherwise
   */
  virtual bool handleBit(int action, int bit, uint8_t &val) = 0;
};

#endif