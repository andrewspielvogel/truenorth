/**
 * @file
 * @date Feb 2018
 * @author Andrew Spielvogel (andrewspielvogel@gmail.com)
 * @brief Implementation of mems_bias.h.
 *
 */

#include <math.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>
#include <truenorth/mems_bias.h>
#include <truenorth/helper_funcs.h>


/*
 *
 * SEE HEADER FILE FOR DOCUMENTATION
 *
 */

MEMSBias::MEMSBias(Eigen::VectorXd k, int hz) 
{
  
  // estimator gains
  kg_ = k(0);
  kn_ = k(1);
  ka_ = k(2);
  km_ = k(3);
  kab_ = k(4);
  kwb_ = k(5);
  kmb_ = k(6);

  
  printf("USING GAINS:\n");
  printf("kg: %f\n",kg_);
  printf("km: %f\n",km_);
  printf("kab: %f\n",kab_);
  printf("kwb: %f\n",kwb_);
  printf("kmb: %f\n",kmb_);
  
  hz_ = hz;
  
  a_b <<0,0,0;
  w_b <<0,0,0;
  m_b <<0,0,0;

  Rni << 1,0,0,0,1,0,0,0,1;


  // initialize usefull vectors
  double earthrate = 7.292150/100000.0;
  Eigen::Matrix3d R_en = get_R_en(lat_);

  Eigen::Vector3d g_e(cos(lat_),0,sin(lat_));
  Eigen::Vector3d w_E(0,0,earthrate);
  Eigen::Vector3d a_e = g_e + skew(w_E)*skew(w_E)*g_e*6371.0*1000.0/9.81;

  a_n_  = R_en.transpose()*a_e;
  Eigen::Vector3d m_n(0.205796,-0.040654,0.468785);

  m_n_  = m_n;

}

MEMSBias::~MEMSBias(void)
{
}

void MEMSBias::step(Eigen::Vector3d ang,Eigen::Vector3d acc, Eigen::Vector3d mag,float dt, float t)
{

  if (!start_)
    {

      acc_hat = acc;
      mag_hat = mag;
      
    }

  
  /**************************************************************
   * Sensor Bias Estimator
   **************************************************************/
  Eigen::Matrix3d kab;
  kab << kab_,0,0,0,kab_,0,0,0,10*kab_;

  Eigen::Matrix3d kmb;
  kmb << kmb_,0,0,0,kmb_,0,0,0,10*kmb_;

  
  Eigen::Vector3d da          = acc_hat - acc;
  Eigen::Vector3d dm          = mag_hat - mag;
  Eigen::Vector3d acc_hat_dot = -skew(ang)*(acc_hat - a_b) + skew(w_b)*acc_hat - ka_*da;
  Eigen::Vector3d mag_hat_dot = -skew(ang - w_b)*mag_hat + skew(ang)*m_b - km_*dm;
  Eigen::Vector3d w_b_dot     = -kwb_*(skew(acc)*da + skew(mag)*dm);
  Eigen::Vector3d a_b_dot     = kab*skew(ang)*da;
  Eigen::Vector3d m_b_dot     = kmb*skew(ang)*dm;


  acc_hat = acc_hat + dt*acc_hat_dot;
  mag_hat = mag_hat + dt*mag_hat_dot;
  w_b     = w_b     + dt*w_b_dot;
  a_b     = a_b     + dt*a_b_dot;
  m_b     = m_b     + dt*m_b_dot;




  /**************************************************************
   * Attitude Estimator
   **************************************************************/
    
  Eigen::Matrix3d P = Rni.transpose()*a_n_.normalized()*a_n_.normalized().transpose()*Rni;
  
  // Define local level (g_error_) and heading (h_error_) error terms
  Eigen::Vector3d g_error = kg_*skew((acc_hat-a_b).normalized())*Rni.transpose()*a_n_.normalized();
  Eigen::Vector3d h_error = P*kn_*skew((mag_hat-m_b).normalized())*Rni.transpose()*m_n_.normalized();

  Rni =  Rni*((skew(g_error + h_error + ang - w_b)*dt).exp());

  
}
