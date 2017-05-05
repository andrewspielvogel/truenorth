/**
 * @file
 * @date Jan. 2017
 * @author Andrew Spielvogel (andrewspielvogel@gmail.com)
 * @brief Implementation of bias_est.h.
 *
 */

#include <math.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <truenorth/helper_funcs.h>
#include <truenorth/bias_est.h>


/*
 *
 * SEE HEADER FILE FOR DOCUMENTATION
 *
 */

BiasEst::BiasEst(Eigen::VectorXd k, float lat)
{
  kg_ = k(0);
  kw_ = k(1);
  ka_ = k(2);
  kz_ = k(3);


  float earth_rate = 15.04*M_PI/180.0/3600.0;
  float earth_radius = 6371.0*1000.0;
  float g_0 = 9.81;
  Eigen::Vector3d w_e(0,0,earth_rate);
  Eigen::Vector3d a_e(cos(lat)-earth_rate*earth_rate*cos(lat)*earth_radius/g_0,0,sin(lat));
  Eigen::Vector3d e_e = skew(w_e)*a_e;
  
  e_n_ = get_R_en(lat).transpose()*e_e;
  a_n_ = get_R_en(lat).transpose()*a_e;

  a_hat << 0,0,0;
  prev_acc_ << 0,0,0;

  w_b << 0,0,0;
  a_b << 0,0,0;
  z << 0,0,0;

}

BiasEst::~BiasEst(void)
{
}

void BiasEst::step(Eigen::Matrix3d Rni, Eigen::Vector3d ang,Eigen::Vector3d acc, float dt)
{

  Eigen::Vector3d da_dt = (acc - prev_acc_)/dt;
  prev_acc_ = acc;
  Eigen::Vector3d da = a_hat - acc + Rni.transpose()*a_n_;
  // Eigen::Vector3d da = a_hat - acc;
  // Eigen::Vector3d a_dot = Rni.transpose()*e_n_ - skew(ang)*a_hat +skew(w_b)*a_hat + skew(ang)*a_b - z - kg_*da;
  // Eigen::Vector3d w_b_dot = -kw_*skew(acc)*da;
  // Eigen::Vector3d a_b_dot = ka_*skew(ang)*da;
  // Eigen::Vector3d z_dot   = kz_*da;

  // a_hat = a_hat + a_dot*dt;
  // w_b   = w_b   + w_b_dot*dt;
  // a_b   = a_b   + a_b_dot*dt;
  // z     = z     + z_dot*dt;


  Eigen::Vector3d a_dot = skew(ang-w_b)*Rni.transpose()*a_n_ - Rni.transpose()*e_n_ + da_dt - kg_*da;
  Eigen::Vector3d w_b_dot = kw_*skew(Rni.transpose()*a_n_)*da;


  a_hat = a_hat + a_dot*dt;
  w_b   = w_b   + w_b_dot*dt;

  float a = 0.99995;
  a_b = w_b_dot;//a_b*a + (1-a)*a_hat;
  

}
