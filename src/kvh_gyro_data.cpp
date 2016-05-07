/*
 * kvh_gyro_data.cpp
 * implementation of kvh_gyro_data.h
 * class for gyro data for KVH 1775 IMU
 *
 * created May 2016
 * Andrew Spielvogel
 * andrewspielvogel@gmail.com
 */

#include <iostream>
#include "ros/ros.h"
#include <eigen/Eigen/Core>
#include <eigen/Eigen/Geometry>
#include <kvh_1775/kvh_gyro_data.h>
#include <ctime>


GyroData::GyroData(float k1_,float k2_,float k3_, float k4_)
{
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
  prev_time = ros::Time::now().toSec();

  // assign gains for bias estimation
  k1 = k1_;
  k2 = k2_;
  k3 = k3_;
  k4 = k4_;

  // initialize initial bias fields
  bias_acc = zero_init;
  bias_ang = zero_init;
  bias_z = zero_init;


  // get current time to name log file
  time_t now = time(0);
  tm *time = localtime(&now);

  int year = 1900 +time->tm_year;
  int month = 1 + time->tm_mon;
  int day = time->tm_mday;
  int hour = time->tm_hour;
  int minute = 1 + time->tm_min;

  char file_name [50];
  sprintf(file_name,"/var/log/KVH/%d_%d_%d_%d_%d.KVH",year,month,day,hour,minute);
  
  // open log file
  fp_ = fopen(file_name,"w");

}

GyroData::~GyroData(void)
{

  fclose(fp_);

}

// set kvh 1775 data packet values
void GyroData::set_values(Eigen::Vector3d a, Eigen::Vector3d w, float m_t, std::vector<bool> stat, unsigned int num)
{
    int skipped = abs(seq_num-num);

    // check for lost data packets
    if (skipped>1&&skipped<127)
    {

	ROS_WARN("Lost %u data packets",skipped);

    }

    // set values
    ang = w;
    acc = a;
    seq_num = num;
    status = stat;
    prev_time = ros::Time::now().toSec();


    // choose which data was sent
    // mod == 0 -- temp
    // mod == 1 -- magx
    // mod == 2 -- magy
    // mod == 3 -- magz
    int mod = num % 4;
    
    if (mod == 0)
      {

	temp = m_t;

      }
    else if (mod == 1)
      {

	mag(0) = m_t;

      }
    else if (mod == 2)
      {

	mag(1) = m_t;

      }
    else if (mod == 3)
      {

	mag(2) = m_t;

      }   
    

    //log data
    fprintf(fp_,"RAW, %f,%f,%f, %f,%f,%f, %f,%f,%f, %f, %d, %f \n",ang(0),ang(1),ang(2),acc(0),acc(1),acc(2),mag(0),mag(1),mag(2),temp,seq_num,prev_time);

}

void GyroData::est_bias()
{
  
  if(seq_num>200)
  {

    acc_est = acc;

  }
  else
  {
    // get dt and da
    double dt = 1.0/1000.0; //ros::Time::now().toSec() - prev_time;
    Eigen::Vector3d da = acc_est - acc;

    // calculate dx
    Eigen::Vector3d da_est = -ang.cross(acc_est) + ang.cross(bias_acc) - acc.cross(bias_ang) - bias_z - k1*da;
    Eigen::Vector3d dab = k2*ang.cross(da);
    Eigen::Vector3d dwb = -k3*acc.cross(da);
    Eigen::Vector3d dzb = k4*da;
 
    // calculate next bias estimate 
    acc_est = acc_est + dt*da_est;
    bias_acc = bias_acc + dt*dab;
    bias_ang = bias_ang + dt*dwb;
    bias_z = bias_z + dt*dzb;

    }

}
