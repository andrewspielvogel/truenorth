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
GyroData::GyroData(std::string log_location, float hz_)
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
  
  
  // get current time to name log file
  time_t now = time(0);
  tm *time = localtime(&now);

  int year = 1900 +time->tm_year;
  int month = 1 + time->tm_mon;
  int day = time->tm_mday;
  int hour = time->tm_hour;
  int minute = 1 + time->tm_min;

  char file_name [128];
  sprintf(file_name,"%s%d_%d_%d_%d_%d.KVH",log_location.c_str(),year,month,day,hour,minute);
  
  // open log file
  fp_ = fopen(file_name,"w");

}

// Destructor
GyroData::~GyroData(void)
{

  fclose(fp_);

}

// set kvh 1775 data packet values
void GyroData::log()
{

    //log data
    fprintf(fp_,"IMU_RAW, %.40f,%.40f,%.40f, %.35f,%.35f,%.35f, %.30f,%.30f,%.30f, %f, %d, %.30f, %d, %d, %d, %d, %d, %d \n",
	    ang(0),ang(1),ang(2),acc(0),acc(1),acc(2),mag(0),mag(1),mag(2),temp,seq_num,timestamp,(int) status.at(0),
	    (int) status.at(1),(int) status.at(2),(int) status.at(3),(int) status.at(4),(int) status.at(5));

}


