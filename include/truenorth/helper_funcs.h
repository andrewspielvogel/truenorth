/**
 * @file
 * @date May 2016
 * @author Andrew Spielvogel (andrewspielvogel@gmail.com)
 * @brief Helper functions
 *
 */


#ifndef HELPER_FUNCS_H
#define HELPER_FUNCS_H

#include <math.h>  
#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>
#include <stdlib.h>
#include <boost/tokenizer.hpp>
#include <boost/foreach.hpp>
#include <string>



/**
 * @brief Skew function.
 * 
 * Skew operator on \f${\rm I\!R}^3\f$.
 * @param w Input vector.
 */
inline Eigen::Matrix3d skew(Eigen::Vector3d w)
{

  Eigen::Matrix3d w_hat;
  w_hat << 0.0,-w(2),w(1),w(2),0.0,-w(0),-w(1),w(0),0.0;
  
  return w_hat;

}

/**
 * @brief Rotation to roll, pitch, heading euler angles.
 * @param R Input rotation.
 */
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

/**
 * @brief Parse param strings.
 * @param param_str parameter string parsed.
 */
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


/**
 * @brief Earth to NED rotation.
 * @param lat Latitude of instrument.
 */
inline Eigen::Matrix3d rot_earth2ned(float lat)
{

  Eigen::Matrix3d R;
  R << -sin(lat),0,-cos(lat),0,1,0,cos(lat),0,-sin(lat);
  return R;

}

/**
 * @brief Matrix exponential.
 * @param R Input matrix.
 */
inline Eigen::Matrix3d mat_exp(Eigen::Matrix3d R)
{

  Eigen::MatrixExponential<Eigen::Matrix3d> exp_inst(R);
  Eigen::Matrix3d R_exp;
  exp_inst.compute(R_exp);

  return R_exp;

}


/**
 * @brief Returns Star to Earth rotation
 * @param t Time (seconds)
 */
inline Eigen::Matrix3d get_R_se(float t)
{

  float rate = 15*M_PI/180/3600;

  Eigen::Vector3d w(0,0,1.0);

  Eigen::Matrix3d R_se = mat_exp(skew(w)*rate*t);

  return R_se;
}

/**
 * @brief Returns Earth to NED rotation
 * @param lat Latitude (radians)
 */
inline Eigen::Matrix3d get_R_en(float lat)
{

  Eigen::Matrix3d R_en;

  R_en << -sin(lat),0,-cos(lat),0,1,0,cos(lat),0,-sin(lat);

  return R_en;

}

/**
 * @brief Returns Star to NED rotation
 * @param lat Latitude (radians)
 * @param t Time (seconds)
 */
inline Eigen::Matrix3d get_R_sn(float lat, float t)
{

  Eigen::Matrix3d R_en = get_R_en(lat);
  
  Eigen::Matrix3d R_se = get_R_se(t);

  Eigen::Matrix3d R_sn = R_se*R_en;

  return R_sn;


}

#endif
