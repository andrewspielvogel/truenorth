/*
 * andrews_func.h
 * Andrew's helper functions
 *
 * created May 2016
 * Andrew Spielvogel
 * andrewspielvogel@gmail.com
 */


#ifndef ANDREWS_FUNC_H
#define ANDREWS_FUNC_H

#include <math.h>  
#include <eigen/Eigen/Core>



// helper skew function
Eigen::Matrix3d skew(Eigen::Vector3d w)
{

  Eigen::Matrix3d w_hat;

  w_hat << 0.0,-w(2),w(1),w(2),0.0,-w(0),-w(1),w(0),0,0;
  
  return w_hat;

}

// helper function for roll, pitch, heading
Eigen::Vector3d rot2rph(Eigen::Matrix3d R)
{

  double h = atan2(R(1,0),R(0,0));
  double ch = cos(h);
  double sh = sin(h);
  double p = atan2(-R(2,0), R(0,0)*ch + R(1,0)*sh);
  double r = atan2(R(0,2)*sh - R(1,2)*ch, -R(0,1)*sh + R(1,1)*ch);

  Eigen::Vector3d rph(r,p,h);

  return rph;

}


#endif
