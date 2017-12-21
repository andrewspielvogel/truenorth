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
#include <Eigen/Core>
#include <unsupported/Eigen/MatrixFunctions>



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
 * @brief Unskew function.
 *
 * @param w_hat Input so(3) matrix.
 */
inline Eigen::Vector3d unskew(Eigen::Matrix3d w_hat)
{

  Eigen::Vector3d w(-w_hat(1,2),w_hat(0,2),-w_hat(0,1));

  return w;

}

/**
 * @brief Rotation around x-axis.
 * @param x angle (units: radians).
 */
inline Eigen::Matrix3d Rx(double x)
{

  Eigen::Matrix3d Rx;
  Rx << 1,0,0,0,cos(x),-sin(x),0,sin(x),cos(x);

  return Rx;

}

/**
 * @brief Rotation around y-axis.
 * @param y angle (units: radians).
 */
inline Eigen::Matrix3d Ry(double y)
{

  Eigen::Matrix3d Ry;
  Ry << cos(y),0,sin(y),0,1,0,-sin(y),0,cos(y);

  return Ry;

}

/**
 * @brief Rotation around z-axis.
 * @param z angle (units: radians).
 */
inline Eigen::Matrix3d Rz(double z)
{

  Eigen::Matrix3d Rz;
  Rz << cos(z),-sin(z),0,sin(z),cos(z),0,0,0,1;

  return Rz;

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
 * @brief Roll, pitch, yaw euler angles to rotation.
 * @param rpy Roll, pitch, yaw vector (units: radians).
 */

inline Eigen::Matrix3d rpy2rot(Eigen::Vector3d rpy)
{

  return Rz(rpy(2))*Ry(rpy(1))*Rx(rpy(0));
  
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
 * @brief Returns Star to Earth rotation
 * @param t Time (seconds)
 */
inline Eigen::Matrix3d get_R_se(float t)
{

  float rate = 15.041*M_PI/180/3600;

  Eigen::Vector3d w(0,0,1.0);

  Eigen::Matrix3d w_hat = skew(w)*rate*t;

  Eigen::Matrix3d R_se = w_hat.exp();

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
