/**
 * @file
 * @date May 2016
 * @author Andrew Spielvogel (andrewspielvogel@gmail.com)
 * @brief Implementation of att_est.h.
 *
 */

#include <math.h>
#include <Eigen/Core>
#include <truenorth/helper_funcs.h>
#include <truenorth/att_est.h>
#include <truenorth/so3_att.h>



/*
 *
 * SEE HEADER FILE FOR DOCUMENTATION
 *
 */

AttEst::AttEst(Eigen::VectorXd k,Eigen::Matrix3d R_align, float lat) : att_(k.head(2),R_align,lat)
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
  printf("kE: %.10f\n",kE_);
  printf("kb: %.10f\n",kb_);
  printf("kab: %f\n",kab_);
  
  lat_ = lat;

  Eigen::Vector3d g_e(cos(lat_),0,sin(lat_));
  Eigen::Vector3d w_E(0,0,7.292150/100000.0);
  Eigen::Vector3d w_E_n = get_R_en(lat_).transpose()*w_E;
  Eigen::Vector3d a_e = g_e + skew(w_E)*skew(w_E)*g_e*6371.0*1000.0/9.81;

  gamma_ = fabs(w_E_n(2))/a_e.norm();

  R_ni = R_align;

  a_b <<0,0,0;
  w_b <<0,0,0;

  w_E_north = R_ni.transpose().block<3,2>(0,0)*w_E_n.block<2,1>(0,0);

  acc_hat = R_ni.transpose()*a_n;
  start_ = 0;


}

AttEst::~AttEst(void)
{
}

void AttEst::step(Eigen::Vector3d ang,Eigen::Vector3d acc, Eigen::Vector3d mag,float dt)
{
  
  if (dt == 0 )
  {
    return;
  }

  // wait until you have a full mag reading to give an initial guess of the heading
  if (start_<4)
  {

    acc_hat = acc - a_b;
    start_ += 1;

    return;

  }


  
  /**************************************************************
   * Sensor Bias and North Vector Estimator
   **************************************************************/



  Eigen::Matrix3d Kab;
  Kab<<kab_,0,0,0,kab_,0,0,0,10*kab_;

  Eigen::Matrix3d Kwb;
  Kwb<<kb_,0,0,0,kb_,0,0,0,5*kb_;
  
  Eigen::Vector3d dacc_hat   = -skew(ang - w_b - w_E_north)*acc_hat + skew(ang)*a_b - ka_*(acc_hat - acc);
  Eigen::Vector3d dw_E_north = -skew(ang - gamma_*acc)*w_E_north - kE_*skew(acc)*acc_hat;
  //Eigen::Vector3d dw_E_north = -skew(ang - gamma_*acc)*w_E_north - (Eigen::MatrixXd::Identity(3,3) - w_E_north.normalized()*w_E_north.normalized().transpose())*kE_*skew(acc)*acc_hat - w_E_north.normalized()*(w_E_north.norm() - w_E_n_(0));
  //Eigen::Vector3d dw_E_north = -skew(ang - gamma_*acc)*w_E_north - kE_*skew(acc)*acc_hat - w_E_north.normalized()*(w_E_north.norm() - w_E_n_(0));
Eigen::Vector3d dw_b       = -Kwb*skew(acc)*acc_hat;  
  Eigen::Vector3d da_b       = Kab*skew(ang)*(acc_hat-acc);

  acc_hat   = acc_hat   + dt*dacc_hat;
  w_E_north = w_E_north + dt*dw_E_north;  
  w_b       = w_b       + dt*dw_b;
  a_b       = a_b       + dt*da_b;

  
  /**************************************************************
   * Attitude Estimator
   **************************************************************/

  
  att_.step(ang-w_b,acc_hat-a_b,w_E_north,dt);
  R_ni = att_.R_ni;
  
}
