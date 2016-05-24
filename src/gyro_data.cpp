/*
 * gyro_data.cpp
 * implementation of gyro_data.h
 * class for gyro data
 *
 * created May 2016
 * Andrew Spielvogel
 * andrewspielvogel@gmail.com
 */

#include <iostream>
#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>
#include <kvh_1775/gyro_data.h>
#include <kvh_1775/helper_funcs.h>
#include <ctime>
#include <string>


GyroData::GyroData(float k1_,float k2_,float k3_, float k4_,float k5_, Eigen::Matrix3d align_, std::string log_location_)
{
  // define inialization values
  Eigen::Vector3d zero_init(0.0,0.0,0.0);
  std::vector<bool> init_stat(6,false);
  Eigen::Matrix3d init_mat;
  init_mat << 1,0,0,0,1,0,0,0,1;

  //initialize est_att
  Rbar_ = init_mat;
  Rd_ = init_mat;
  att = zero_init;

  R_align_ = align_;

  // initialize imu data to zero
  mag = zero_init;
  ang = zero_init;
  acc = zero_init;

  // initialize other fields
  temp = 0.0;
  seq_num = 500;
  status = init_stat;
  prev_time = ros::Time::now().toSec();
  diff = 0.0;
  
  // assign gains for bias estimation
  k1 = k1_;
  k2 = k2_;
  k3 = k3_;
  k4 = k4_;
  k5 = k5_;

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

  char file_name [128];
  sprintf(file_name,"%s%d_%d_%d_%d_%d.KVH",log_location_.c_str(),year,month,day,hour,minute);
  
  // open log file
  fp_ = fopen(file_name,"w");

}

GyroData::~GyroData(void)
{

  fclose(fp_);

}

// set kvh 1775 data packet values
void GyroData::set_values(Eigen::Vector3d a, Eigen::Vector3d w, Eigen::Vector3d m, float temp_, std::vector<bool> stat, unsigned int num)
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
    temp = temp_;
    mag = m;
    

    //log data
    fprintf(fp_,"IMU_RAW, %.40f,%.40f,%.40f, %.35f,%.35f,%.35f, %.30f,%.30f,%.30f, %f, %d, %.30f, %d, %d, %d, %d, %d, %d, %.30f, %.30f,%.30f \n",
	    ang(0),ang(1),ang(2),acc(0),acc(1),acc(2),mag(0),mag(1),mag(2),temp,seq_num,prev_time,(int) status.at(0),
	    (int) status.at(1),(int) status.at(2),(int) status.at(3),(int) status.at(4),(int) status.at(5),att(0),
	    att(1),att(2));

}

// estimate bias
void GyroData::est_bias()
{
  
  if(seq_num>200)
  {

    acc_est = acc;

  }
  else
  {
    // get dt and da
    double dt = diff;
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


// estimate attitude
void GyroData::est_att()
{

  Eigen::Vector3d u = Rd_*acc;
  Eigen::Vector3d y(0.0,0.0,1.0);

  Eigen::Vector3d y_est = Rbar_*u;
  Eigen::Vector3d err = k5*Rbar_.transpose()*y_est.cross(y);
  Eigen::Matrix3d dt_err = diff*skew(err);
  Rbar_ = Rbar_*dt_err.exp();
  
  att = rot2rph(R_align_*Rd_.transpose()*Rbar_.transpose());

}
