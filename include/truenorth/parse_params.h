
/**
 * @file
 * @date November 2017
 * @author Andrew Spielvogel (andrewspielvogel@gmail.com)
 * @brief Parameter parser for attitude estimation rosnode.
 *
 */


#ifndef PARSE_PARAMS_H
#define PARSE_PARAMS_H

#include <ros/ros.h>
#include <Eigen/Core>
#include <string>
#include <math.h>



struct estimator_params {

  int rate;
  int baud;
  int hz;

  float lat;
  
  std::string port;
  std::string log_location;

  Eigen::VectorXd k;

  Eigen::Matrix3d R_align;
  Eigen::Matrix3d R0;

} ;

estimator_params load_params(ros::NodeHandle n) {


  ROS_INFO("Loading Params");
  estimator_params params;

  params.rate = 10;
  n.getParam("/truenorth/rate",params.rate);

  ROS_INFO("rate: %d",params.rate);
  
  params.port = "/dev/ttyUSB0";
  n.getParam("/truenorth/port",params.port);

  ROS_INFO("port: %s",params.port.c_str());

  params.log_location = "/var/log/KVH";
  n.getParam("/truenorth/log_loc",params.log_location);

  ROS_INFO("log_loc: %s",params.log_location.c_str());
  
  // baud rate
  params.baud = 921600;
  n.getParam("/truenorth/baud",params.baud);

  ROS_INFO("baud: %d",params.baud);

  Eigen::Vector3d rpy;

  // instrument alignment matrix
  std::string instr_align = "0,0,0";  // default
  n.getParam("/truenorth/instr_align",instr_align);
  sscanf(instr_align.c_str(),"%lf,%lf,%lf",&rpy(0),&rpy(1),&rpy(2));
  params.R_align = rpy2rot(rpy);

  
  // Rni(t0) matrix
  std::string R0_ = "0,0,0";  // default
  n.getParam("/truenorth/R0",R0_);
  sscanf(R0_.c_str(),"%lf,%lf,%lf",&rpy(0),&rpy(1),&rpy(2));
  params.R0 = rpy2rot(rpy);
    
  // estimation gains
  std::string gains = "0.01,0.02,0.0,1.0,0.01,0.0,0.0";  // default
  n.getParam("/truenorth/gains",gains);
  Eigen::VectorXd k(7);
  sscanf(gains.c_str(),"%lf,%lf,%lf,%lf,%lf,%lf,%lf",&k(0),&k(1),&k(2),&k(3),&k(4),&k(5),&k(6));
  params.k = k;
  
  // latitude
  double lat_input = 39.32;
  n.getParam("/truenorth/latitude",lat_input);
  params.lat = ((float) lat_input)*M_PI/180;

  ROS_INFO("latitude: %f",params.lat);
  
  // sample rate
  params.hz = 1000; // default
  n.getParam("/truenorth/hz",params.hz);

  ROS_INFO("hz: %d",params.hz);
  
  return params;
  
}

#endif
