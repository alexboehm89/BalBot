/**
 * @file imu_9dof.cpp
 * @author Alexander Boehm (alex.boehm89@gmail.com)
 * @brief              Description
 * @version 0.1
 * @date 2020-04-17
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#include "adafruit_9dof_imu/imu_9dof.h"

IMU9DOF::IMU9DOF()
{
  // Read settings from parameter server
  nh_.param<int>("/imu/update_rate", imu_rate_, 100);
  nh_.param<int>("/imu/gyro/range", gyro_params_.range, 250);
  nh_.param<int>("/imu/gyro/rate", gyro_params_.rate, 760);
  nh_.param<bool>("/imu/gyro/autorange", gyro_params_.isAutoRange, false);
  nh_.param<int>("/imu/accel/range", accel_params_.range, 2);
  nh_.param<int>("/imu/accel/rate", accel_params_.rate, 400);

  // Set up sensors
  if (!gyro_.setup(gyro_params_))
  {
    ROS_ERROR("Gyroscope: No L3GD20 detected. Check wiring!");
  }
  if (!accel_.setup(accel_params_))
  {
    ROS_ERROR("Accelerometer: No LSM303 detected. Check wiring!");
  }

  // Publisher
  imu_pub_ = nh_.advertise<sensor_msgs::Imu>("imu/data_raw", 1000);

  // Set up services to change output data range of sensors
  gyro_range_srv_  = nh_.advertiseService("/imu/gyro/set_range", 
                                 &IMU9DOF::setRangeGyro, 
                                 this);
  accel_range_srv_ = nh_.advertiseService("/imu/accel/set_range", 
                                 &IMU9DOF::setRangeAccel, 
                                 this);
  
  // Set up services to change output data rate of gyroscope
  gyro_rate_srv_  = nh_.advertiseService("/imu/gyro/set_rate", 
                                 &IMU9DOF::setRateGyro, 
                                 this);
  accel_rate_srv_ = nh_.advertiseService("/imu/accel/set_rate", 
                                 &IMU9DOF::setRateAccel, 
                                 this);

  // Set up services to change operating mode of gyroscope
  gyro_power_down_srv_   = nh_.advertiseService("/imu/gyro/power_down", 
                                 &IMU9DOF::powerDownGyro, 
                                 this);
  gyro_sleep_srv_        = nh_.advertiseService("/imu/gyro/sleep", 
                                  &IMU9DOF::sleepGyro, 
                                  this);
  gyro_normal_mode_srv_  = nh_.advertiseService("/imu/gyro/normal_mode", 
                                  &IMU9DOF::normalModeGyro, 
                                  this);
  accel_power_down_srv_  = nh_.advertiseService("/imu/accel/power_down", 
                                  &IMU9DOF::powerDownAccel, 
                                  this);
  accel_low_power_srv_   = nh_.advertiseService("/imu/accel/low_power", 
                                  &IMU9DOF::lowPowerAccel, 
                                  this);
  accel_normal_mode_srv_ = nh_.advertiseService("/imu/accel/normal_mode", 
                                  &IMU9DOF::normalModeAccel, 
                                  this);
  
  // Set up services to reset sensors
  gyro_reset_srv_  = nh_.advertiseService("/imu/gyro/reset", 
                                 &IMU9DOF::resetGyro, 
                                 this);
  accel_reset_srv_ = nh_.advertiseService("/imu/accel/reset", 
                                 &IMU9DOF::resetAccel, 
                                 this);

  // Prepare IMU message
  imu_msg_.header.frame_id = "imu";
}

IMU9DOF::~IMU9DOF()
{
}


void IMU9DOF::run()
{
  ros::Rate loop_rate(imu_rate_);

  // Loop for IMU 
  while (ros::ok())
  {
    // Get data from sensors
    gyro_.getData(gyro_data_);
    accel_.getData(accel_data_);

    // Create IMU message
    imu_msg_.header.stamp          = ros::Time::now();
    imu_msg_.angular_velocity.x    = gyro_data_.x;
    imu_msg_.angular_velocity.y    = gyro_data_.y;
    imu_msg_.angular_velocity.z    = gyro_data_.z;
    imu_msg_.linear_acceleration.x = accel_data_.x;
    imu_msg_.linear_acceleration.y = accel_data_.y;
    imu_msg_.linear_acceleration.z = accel_data_.z;

    // Publish message
    imu_pub_.publish(imu_msg_);

    // Handle ROS events
    ros::spinOnce();

    // loop rate loop rate if posible
    if (!loop_rate.sleep())
    {
      // Print warning
      ROS_WARN("Update rate of imu_node too high.");
    }
  }
}


bool IMU9DOF::setRangeGyro(adafruit_9dof_imu::SetRange::Request  &req,
                           adafruit_9dof_imu::SetRange::Response &res)
{
  // Set range of gyro
  res.success = gyro_.setRange(req.range);

  // Inform user about result
  if (res.success)
  {
    res.message = "Gyroscope: Output data range set.";
  } 
  else
  {
    res.message = "Gyroscope: Output data range not set. Valid ranges: 250|500|2000 DPS";
  }

  return true;
}

bool IMU9DOF::setRangeAccel(adafruit_9dof_imu::SetRange::Request  &req,
                            adafruit_9dof_imu::SetRange::Response &res)
{
  // Set range of accelerometer
  res.success = accel_.setRange(req.range);

  // Inform user about result
  if (res.success)
  {
    res.message = "Accelerometer: Output data range set.";
  } 
  else
  {
    res.message = "Accelerometer: Output data range not set. Valid ranges: 2|4|8|16 g";
  }

  return true;
}

bool IMU9DOF::setRateGyro(adafruit_9dof_imu::SetRate::Request  &req,
                          adafruit_9dof_imu::SetRate::Response &res)
{
  // Set rate of gyro
  res.success =  gyro_.setRate(req.rate);

  // Inform user about result
  if (res.success)
  {
    res.message = "Gyroscope: Output data rate set.";
  } 
  else
  {
    res.message = "Gyroscope: Output data rate not set. Valid ranges: 95|190|380|760 Hz";
  }
  
  return true;
}

bool IMU9DOF::setRateAccel(adafruit_9dof_imu::SetRate::Request  &req,
                           adafruit_9dof_imu::SetRate::Response &res)
{
  // Set rate of accelerometer
  res.success =  accel_.setRate(req.rate);

  // Inform user about result
  if (res.success)
  {
    res.message = "Accelerometer: Output data rate set.";
  } 
  else
  {
    res.message = "Accelerometer: Output data rate not set." 
                  "Valid ranges: 1|10|25|50|100|200|400 Hz";
  }

  return true;
}

bool IMU9DOF::powerDownGyro(std_srvs::Trigger::Request  &req,
                            std_srvs::Trigger::Response &res)
{
  // Power down gyroscope and indicate success
  res.success = gyro_.powerDown();
  res.message = "Gyroscope: Power down mode enabled.";

  return true;
}


bool IMU9DOF::sleepGyro(std_srvs::Trigger::Request  &req,
                        std_srvs::Trigger::Response &res)
{
  // Set gyroscope to sleep mode and indicate success
  res.success = gyro_.sleep();
  res.message = "Gyroscope: Sleep mode enabled.";

  return true;
}


bool IMU9DOF::normalModeGyro(std_srvs::Trigger::Request  &req,
                             std_srvs::Trigger::Response &res)
{
  // Set gyroscope to normal mode and indicate success
  res.success = gyro_.setNormalMode();
  res.message = "Gyroscope: Normal mode enabled.";

  return true;
}

bool IMU9DOF::powerDownAccel(std_srvs::Trigger::Request  &req,
                             std_srvs::Trigger::Response &res)
{
  // Power down accelerometer and indicate success
  res.success = accel_.powerDown();
  res.message = "Accelerometer: Power down mode enabled.";

  return true;
}

bool IMU9DOF::lowPowerAccel(std_srvs::Trigger::Request  &req,
                            std_srvs::Trigger::Response &res)
{
  // Set accelerometer to low power mode and indicate success
  res.success = accel_.lowPowerMode();
  res.message = "Accelerometer: Low power mode enabled.";

  return true;
}

bool IMU9DOF::normalModeAccel(std_srvs::Trigger::Request  &req,
                               std_srvs::Trigger::Response &res)
{
  // Set accelerometer to normal mode and indicate success
  res.success = accel_.setNormalMode();
  res.message = "Accelerometer: Normal mode enabled.";

  return true;
}

bool IMU9DOF::resetGyro(std_srvs::Trigger::Request  &req,
                        std_srvs::Trigger::Response &res)
{
  // Reset gyroscope to default parameters and indicate success
  res.success = gyro_.reset();
  res.message = "Gyroscope: Reset to default parameters and normal mode enabled.";

  return true;
}

bool IMU9DOF::resetAccel(std_srvs::Trigger::Request  &req,
                         std_srvs::Trigger::Response &res)
{
  // Reset accelerometer to default parameters and indicate success
  res.success = accel_.reset();
  res.message = "Accelerometer: Reset to default parameters and normal mode enabled.";

  return true;
}
