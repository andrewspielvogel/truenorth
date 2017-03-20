/**
 * @file
 * @date May 2016
 * @author Andrew Spielvogel (andrewspielvogel@gmail.com)
 * 
 * @brief Implementation of gyro_data.h.
 *
 * Class for gyro data.
 *
 */

#include <iostream>
#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <truenorth/gyro_data.h>
#include <truenorth/helper_funcs.h>
#include <truenorth/att_est.h>
#include <truenorth/bias_est.h>
#include <ctime>
#include <string>

/*
 *
 * SEE HEADER FILE FOR DOCUMENTATION
 *
 */

// Constructor
GyroData::GyroData(float hz_)
{

  hz = hz_;

  // define inialization values
  Eigen::Vector3d zero_init(0.0,0.0,0.0);
  std::vector<bool> init_stat(6,false);
  
  // initialize imu data to zero
  mag = zero_init;
  ang = zero_init;
  acc = zero_init;

  // initialize other fields
  temp = 0.0;
  seq_num = 500;
  status = init_stat;
  timestamp = ros::Time::now().toSec();
  t_start = timestamp;
  diff = 1.0/((double)hz);

}

// Destructor
GyroData::~GyroData(void)
{
}




