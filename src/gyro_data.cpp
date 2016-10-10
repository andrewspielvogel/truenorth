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
#include <unsupported/Eigen/KroneckerProduct>
#include <truenorth/gyro_data.h>
#include <truenorth/helper_funcs.h>
#include <truenorth/att_est.h>
#include <ctime>
#include <string>

/*
 *
 * SEE HEADER FILE FOR DOCUMENTATION
 *
 */

// Constructor
GyroData::GyroData(Eigen::VectorXd k, Eigen::Matrix3d align, std::string log_location, float lat, float hz_):Rbar_(k.tail(4),align, lat, hz)
{
  // define inialization values
  Eigen::Vector3d zero_init(0.0,0.0,0.0);
  std::vector<bool> init_stat(6,false);
  Eigen::Matrix3d init_mat;
  init_mat << 1,0,0,0,1,0,0,0,1;
  Eigen::MatrixXd R_init(9,1);
  R_init << 1,0,0,0,-1,0,0,0,-1;
  // R = align;

  //initialize est_att
  att = zero_init;

  R_align_ = align;
  R = align;
  R.resize(9,1);
  Rd  = init_mat;

  lat_ = lat;

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
  diff = 1.0/((double)hz_);
  
  // assign gains for bias estimation
  k1_ = k(0);
  k2_ = k(1);
  k3_ = k(2);
  k4_ = k(3);
  hz = hz_;

  // initialize initial bias fields
  bias_acc = zero_init;
  bias_ang = zero_init;
  bias_z = zero_init;
  acc_est = zero_init;


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
    fprintf(fp_,"IMU_RAW, %.40f,%.40f,%.40f, %.35f,%.35f,%.35f, %.30f,%.30f,%.30f, %f, %d, %.30f, %d, %d, %d, %d, %d, %d, %.30f, %.30f,%.30f,%.10f,%.10f,%.10f,%.10f,%.10f,%.10f \n",
	    ang(0),ang(1),ang(2),acc(0),acc(1),acc(2),mag(0),mag(1),mag(2),temp,seq_num,timestamp,(int) status.at(0),
	    (int) status.at(1),(int) status.at(2),(int) status.at(3),(int) status.at(4),(int) status.at(5),att(0),
	    att(1),att(2),bias_ang(0),bias_ang(1),bias_ang(2),acc_est(0),acc_est(1),acc_est(2));

}

// estimate bias
void GyroData::est_bias()
{

  // get dt and da
  double dt = diff;
  Eigen::Vector3d da = acc_est - acc;

  Rd = Rd*mat_exp(skew(ang)*dt);

  Eigen::Matrix3d R_sn = get_R_sn(lat_, timestamp-t_start);
  Eigen::Matrix3d R_en = get_R_en(lat_);
  Eigen::Vector3d up_n(0,0,-1);

  // integrate Rd
  Rd = Rd*mat_exp(skew((ang)*dt));
 

  Eigen::Vector3d g_e(cos(lat_),0,sin(lat_));
  Eigen::Vector3d w_e(0,0,15.0*M_PI/180.0/3600.0);
  Eigen::Vector3d a_e = g_e + skew(w_e)*skew(w_e)*g_e*6371*1000/9.81;

  Eigen::Vector3d a_n = R_en.transpose()*a_e;
  Eigen::Vector3d w_n = R_en.transpose()*w_e;

  Eigen::Vector3d e_n = skew(w_n)*a_n;

  Eigen::MatrixXd kronecker = Eigen::kroneckerProduct(R_sn*e_n,Rd);

  Eigen::Vector3d dacc = -skew(ang-bias_ang)*acc + kronecker.transpose()*R - k1_*da;
  Eigen::VectorXd dR   = kronecker*da;
  Eigen::Vector3d dwb  = -skew(acc)*da;

  acc_est  = acc_est + dt*dacc;
  R        = R + dt*k2_*dR;
  bias_ang = bias_ang + dt*k3_*dwb;


}


// estimate attitude
void GyroData::est_att()
{

  Rbar_.step(ang-bias_ang,acc_est-bias_acc,timestamp-t_start,diff);
  
  // get attitude estimation
  att = rot2rph(Rbar_.R_ni*R_align_);


}
