/*
 * helper_funcs.h
 * helper functions
 *
 * created May 2016
 * Andrew Spielvogel
 * andrewspielvogel@gmail.com
 */


#ifndef HELPER_FUNCS_H
#define HELPER_FUNCS_H

#include <math.h>  
#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <stdlib.h>
#include <boost/tokenizer.hpp>
#include <boost/foreach.hpp>
#include <string>



// helper skew function
inline Eigen::Matrix3d skew(Eigen::Vector3d w)
{

  Eigen::Matrix3d w_hat;
  w_hat << 0.0,-w(2),w(1),w(2),0.0,-w(0),-w(1),w(0),0.0;
  
  return w_hat;

}

// helper function to convert a rotation matrix to roll, pitch, heading
inline Eigen::Vector3d rot2rph(Eigen::Matrix3d R)
{

  double h = atan2(R(1,0),R(0,0));
  double ch = cos(h);
  double sh = sin(h);
  double p = atan2(-R(2,0), R(0,0)*ch + R(1,0)*sh);
  double r = atan2(R(0,2)*sh - R(1,2)*ch, -R(0,1)*sh + R(1,1)*ch);

  Eigen::Vector3d rph(r,p,h);

  return rph;

}

// helper function for parsing param strings
inline Eigen::MatrixXd parse_string(std::string param_str)
{

  boost::char_separator<char> sep(",");
  boost::tokenizer<boost::char_separator<char> > align_tokens(param_str,sep);    

  Eigen::MatrixXd parsed_mat(9,1);
  int i = 0;
  BOOST_FOREACH (const std::string& t, align_tokens)
  {

    parsed_mat(i) = strtod(t.c_str(),NULL);  
    i++;

  }
  
  return parsed_mat;

}

#endif
