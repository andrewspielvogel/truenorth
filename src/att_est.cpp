/**
 * @file
 * @date May 2016
 * @author Andrew Spielvogel (andrewspielvogel@gmail.com)
 * @brief Implementation of att_est.h.
 *
 */

#include <math.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>
#include <truenorth/helper_funcs.h>
#include <truenorth/att_est.h>



/*
 *
 * SEE HEADER FILE FOR DOCUMENTATION
 *
 */

AttEst::AttEst(Eigen::VectorXd k,Eigen::Matrix3d R_align, float lat, int hz) 
{
  
  // estimator gains
  kg_ = k(0);
  kw_ = k(1);
  ka_ = k(2);
  kE_ = k(3);
  kb_ = k(4);
  kab_= k(5);

  printf("USING GAINS:\n");
  printf("kg: %f\n",kg_);
  printf("kw: %f\n",kw_);
  printf("ka: %f\n",ka_);
  printf("kE: %f\n",kE_);
  printf("kb: %f\n",kb_);
  printf("kab: %f\n",kab_);
  
  hz_ = hz;
  lat_ = lat;

  // initialize usefull vectors
  double earthrate = 7.292150/100000.0;
  Eigen::Matrix3d R_en = get_R_en(lat_);

  Eigen::Vector3d g_e(cos(lat_),0,sin(lat_));
  Eigen::Vector3d w_E(0,0,earthrate);
  Eigen::Vector3d a_e = g_e + skew(w_E)*skew(w_E)*g_e*6371.0*1000.0/9.81;

  a_n  = R_en.transpose()*a_e;
  P_   = a_n.normalized()*a_n.normalized().transpose();
  R_ni = R_align;

  w_E_n = R_en.transpose()*w_E;
  w_E_n(2) = 0;

  gamma_ = w_E_n(2)/a_n.norm();

  
  wearth_n_ = R_en.transpose()*w_E;

  a_b <<0,0,0;// -0.0008,0,-0.0008;
  w_b <<0,0,0;// 0.00002,0,-0.00001;

  w_E_north = R_ni.transpose()*w_E_n;

  acc_hat = R_ni.transpose()*a_n;
  start_ = 0;

}

AttEst::~AttEst(void)
{
}

void AttEst::step(Eigen::Vector3d ang,Eigen::Vector3d acc, Eigen::Vector3d mag,float dt, float t)
{

  acc = acc*9.81;
  Eigen::Matrix3d I;

  I << 1,0,0,0,1,0,0,0,1;

  
  if (dt == 0 )
  {
    return;
  }

  // wait until you have a full mag reading to give an initial guess of the heading
  if (start_<4)
  {

    t_start_ = t;
    acc_hat = acc - a_b;
    start_ += 1;
    /*
    w_E_north = (I - acc_hat.normalized()*acc_hat.normalized().transpose())*mag.normalized()*w_E_n.norm();

    R_ni.transpose().block<3,1>(0,0) = w_E_north.normalized();
    R_ni.transpose().block<3,1>(0,2) = -acc_hat.normalized();
    R_ni.transpose().block<3,1>(0,1) = (skew(-acc_hat)*w_E_north).normalized();
    */

    return;

  }



  float delay = 1000.0;
  float scale = 0.00001;
  float kE = kE_;//-atan(t-t_start_-delay)*scale/M_PI + scale/2.0 + kE_;

  
  
  /**************************************************************
   * Sensor Bias and North Vector Estimator
   **************************************************************/

  
  Eigen::Matrix3d kab;
  kab << kab_,0,0,0,0*kab_,0,0,0,kab_;
  
  Eigen::Vector3d dacc_hat   = -skew(ang - w_b - w_E_north)*acc_hat + skew(ang)*a_b - ka_*(acc_hat - acc);
  Eigen::Vector3d dw_E_north = -skew(ang + gamma_*acc)*w_E_north - kE*skew(acc)*acc_hat;
  Eigen::Vector3d dw_b       = -kb_*skew(acc)*acc_hat;
  Eigen::Vector3d da_b       = kab_*(I-acc.normalized()*acc.normalized().transpose())*skew(ang)*(acc_hat-acc);
  //Eigen::Vector3d da_b       = kab*skew(ang)*(acc_hat-acc);


  
  acc_hat   = acc_hat   + dt*dacc_hat;
  w_E_north = w_E_north + dt*dw_E_north;
  w_b       = w_b       + dt*dw_b;
  a_b       = a_b       + dt*da_b;
  w_E_north = w_E_n.norm()*(((I-P_)*w_E_north).normalized());
  //w_E_north = w_E_north.normalized()*w_E_n.norm();
  

  /*
  Eigen::Vector3d dacc_hat   = -skew(ang)*(acc_hat-a_b)  - ka_*(acc_hat - acc);
  //da_b       = kab_*skew(ang)*(acc_hat-acc);
  da_b       = kab_*(I-acc.normalized()*acc.normalized().transpose())*skew(ang)*(acc_hat-acc);
  acc_hat   = acc_hat   + dt*dacc_hat;
  a_b       = a_b       + dt*da_b;
  */
  
  /**************************************************************
   * Attitude Estimator
   **************************************************************/
    
  P_ = R_ni.transpose()*a_n.normalized()*a_n.normalized().transpose()*R_ni;
  
  
  // Define local level (g_error_) and heading (h_error_) error terms
  //g_error_ = kg_*skew(acc_hat-a_b)*R_ni.transpose()*a_n;
  g_error_ = kg_*skew((acc_hat-a_b).normalized())*R_ni.transpose()*a_n.normalized();
  h_error_ = P_*kw_*skew(w_E_north.normalized())*R_ni.transpose()*w_E_n.normalized();
  //h_error_ = kw_*skew(w_E_north.normalized())*R_ni.transpose()*w_E_n.normalized();

  R_ni =  R_ni*((skew(g_error_ + h_error_ + ang - R_ni.transpose()*wearth_n_)*dt).exp());

  //R_ni =  R_ni*((skew(g_error_ + ang)*dt).exp());
  
}
