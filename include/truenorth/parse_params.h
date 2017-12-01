
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

  estimator_params params;

  params.rate = 10;
  n.getParam("rate",params.rate);

  params.port = "/dev/ttyUSB0";
  n.getParam("port",params.port);

  params.log_location = "/var/log/KVH";
  n.getParam("log_loc",params.log_location);

  // baud rate
  params.baud = 921600;
  n.getParam("baud",params.baud);

  Eigen::Vector3d rpy;

  // instrument alignment matrix
  std::string instr_align = "0,0,0";  // default
  n.getParam("instr_align",instr_align);
  sscanf(instr_align.c_str(),"%lf,%lf,%lf",&rpy(0),&rpy(1),&rpy(2));
  params.R_align = rpy2rot(rpy);

  // Rni(t0) matrix
  std::string R0_ = "0,0,0";  // default
  n.getParam("R0",R0_);
  sscanf(R0_.c_str(),"%lf,%lf,%lf",&rpy(0),&rpy(1),&rpy(2));
  params.R0 = rpy2rot(rpy);
    
  // estimation gains
  std::string gains = "0.01,0.02,0.0,1.0,0.01,0.0,0.0";  // default
  n.getParam("gains",gains);
  Eigen::VectorXd k(7);
  sscanf(gains.c_str(),"%lf,%lf,%lf,%lf,%lf,%lf,%lf",&k(0),&k(1),&k(2),&k(3),&k(4),&k(5),&k(6));
  params.k = k;
  
  // latitude
  double lat_input = 39.32;
  n.getParam("latitude",lat_input);
  params.lat = ((float) lat_input)*M_PI/180;

  // sample rate
  params.hz = 1000; // default
  n.getParam("hz",params.hz);
  
  return params;
  
}

#endif
