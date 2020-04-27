/**
 * @file imu_9dof.h
 * @author Alexander Boehm (alex.boehm89@gmail.com)
 * @brief              Description
 * @version 0.1
 * @date 2020-04-17
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#ifndef IMU_9DOF_H
#define IMU_9DOF_H

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_srvs/Trigger.h>
#include "adafruit_9dof_imu/gyroscope_L3GD20.h"
#include "adafruit_9dof_imu/accelerometer_LSM303.h"
#include "adafruit_9dof_imu/SetRate.h"
#include "adafruit_9dof_imu/SetRange.h"

class IMU9DOF
{
public:
  /**
   * @brief  Constructor
   * 
   */
  IMU9DOF();

  /**
   * @brief  Destructor
   * 
   */
  ~IMU9DOF();

  /**
   * @brief  Loop for getting and publishing IMU data
   * 
   */
  void run();

private:
  /**
   * @brief  Service function to set output data range and rate of sensors
   * 
   */
  bool setRangeGyro  (adafruit_9dof_imu::SetRange::Request  &req,
                      adafruit_9dof_imu::SetRange::Response &res);
  bool setRangeAccel (adafruit_9dof_imu::SetRange::Request  &req,
                      adafruit_9dof_imu::SetRange::Response &res);

  /**
   * @brief  Service functions to set output data rate of sensors
   * 
   */
  bool setRateGyro  (adafruit_9dof_imu::SetRate::Request  &req,
                     adafruit_9dof_imu::SetRate::Response &res);
  bool setRateAccel (adafruit_9dof_imu::SetRate::Request  &req,
                     adafruit_9dof_imu::SetRate::Response &res);

  /**
   * @brief  Service functions to set different modes of sensors 
   * 
   */
  bool powerDownGyro  (std_srvs::Trigger::Request   &req,
                       std_srvs::Trigger::Response  &res);
  bool sleepGyro      (std_srvs::Trigger::Request   &req,
                       std_srvs::Trigger::Response  &res);
  bool normalModeGyro (std_srvs::Trigger::Request   &req,
                       std_srvs::Trigger::Response  &res);
  bool powerDownAccel (std_srvs::Trigger::Request   &req,
                       std_srvs::Trigger::Response  &res);
  bool lowPowerAccel  (std_srvs::Trigger::Request   &req,
                       std_srvs::Trigger::Response  &res);
  bool normalModeAccel(std_srvs::Trigger::Request   &req,
                       std_srvs::Trigger::Response  &res);

  /**
   * @brief  Service function to reset sensors to default parameters
   * 
   */
  bool resetGyro (std_srvs::Trigger::Request   &req,
                  std_srvs::Trigger::Response  &res);
  bool resetAccel(std_srvs::Trigger::Request   &req,
                  std_srvs::Trigger::Response  &res);

  // ROS node handle
  ros::NodeHandle nh_;

  // Sensor objects
  GyroscopeL3GD20     gyro_;
  AccelerometerLSM303 accel_;

  // Update rate of main IMU loop
  int imu_rate_;

  // Parameters of IMU sensors
  gyroParams_t gyro_params_;
  accelParams_t accel_params_;

  // Object to save data
  sensorData_t gyro_data_;
  sensorData_t accel_data_;

  // Message for publisher
  sensor_msgs::Imu imu_msg_;

  // Ros publisher
  ros::Publisher imu_pub_;

  // ROS services to set range of sensors
  ros::ServiceServer gyro_range_srv_;
  ros::ServiceServer accel_range_srv_;

  // ROS services to set rate of sensors
  ros::ServiceServer gyro_rate_srv_;
  ros::ServiceServer accel_rate_srv_;

  // ROS services to set different modes of sensors
  ros::ServiceServer gyro_power_down_srv_;
  ros::ServiceServer gyro_sleep_srv_;
  ros::ServiceServer gyro_normal_mode_srv_;
  ros::ServiceServer accel_power_down_srv_;
  ros::ServiceServer accel_low_power_srv_;
  ros::ServiceServer accel_normal_mode_srv_;

  // ROS services to reset sensors
  ros::ServiceServer gyro_reset_srv_;
  ros::ServiceServer accel_reset_srv_;
};


#endif

