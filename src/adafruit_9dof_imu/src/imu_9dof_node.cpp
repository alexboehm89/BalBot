/**
 * @file imu_9dof_node.cpp
 * @author Alexander Boehm (alex.boehm89@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2020-04-12
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#include <ros/ros.h>
#include "adafruit_9dof_imu/imu_9dof.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "imu_9dof_node");

  IMU9DOF imu;

  imu.run();

  return 0;
}
