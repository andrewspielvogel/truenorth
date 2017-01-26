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
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>
#include <stdlib.h>
#include <boost/tokenizer.hpp>
#include <boost/foreach.hpp>
#include <string>
#include <iostream>
#include <fstream>
#include <stdio.h>


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

/**
 * @brief CSV to Matrix
 * 
 * Converts CSV file to Eigen double Matrix
 * @param file File name
 * @param rows Number of rows to load
 * @param cols Number of cols to load
 *
 */
inline Eigen::MatrixXd readCSV(std::string file, int rows, int cols) {

  std::ifstream in(file.c_str());
  
  std::string line;

  int row = 0;

  Eigen::MatrixXd data(rows, cols);

  if (in.is_open()) {

    while (std::getline(in, line)) {

      char *ptr = (char *) line.c_str();

      char*pEnd = ptr;

      for (int col = 0; col < cols; col++) {
	
	while(!isdigit(pEnd[0]) && pEnd[0] != '-'){
	  pEnd = pEnd +1;
	}

      double num = strtod(pEnd,&pEnd);
      data(row,col) = num;
      }

      row++;
      if (!(row < rows))
	break;
    }

    in.close();
  }
  return data;
}

#endif
