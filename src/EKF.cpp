/**
 * @file
 * @date Nov. 2017
 * @author Andrew Spielvogel (andrewspielvogel@gmail.com)
 * @brief Implementation of EKF.h.
 *
 */

#include <math.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <truenorth/helper_funcs.h>
#include <truenorth/EKF.h>



/*
 *
 * SEE HEADER FILE FOR DOCUMENTATION
 *
 */

AttEKF::AttEKF(Eigen::VectorXd x0, Eigen::VectorXd p0, Eigen::VectorXd r0, Eigen::VectorXd q0, float lat)
{

  x = x0;
  P = p0.asDiagonal();
  R = r0.asDiagonal();
  Q = q0.asDiagonal();

  float w_E_mag = 7.292150/100000.0;
  gamma = sin(lat)*w_E_mag;
  w_En_mag = cos(lat)*w_E_mag;

  Eigen::Vector3d a_e(cos(lat)-w_E_mag*w_E_mag*cos(lat)*6371000/9.81,0,sin(lat));
  a_mag = a_e.norm();
  

}



Eigen::VectorXd AttEKF::calc_f(float dt)
{

  Eigen::Vector3d a  = (-skew(x.segment(3,3)-x.segment(6,3)-x.segment(9,3))*dt).exp()*x.head(3);

  Eigen::Vector3d wEn = (-skew(x.segment(3,3)-x.segment(9,3)+gamma*x.head(3).normalized())*dt).exp()*x.segment(6,3);


  Eigen::VectorXd f(12);

  f << a, x.segment(3,3), wEn, x.segment(9,3);

  return f;
  
}


Eigen::MatrixXd AttEKF::dynamics_jacobian(float dt)
{

  Eigen::MatrixXd Fx(12,12);

  Eigen::MatrixXd zeross(3,12);
  zeross.setZero();

  Eigen::Matrix3d Ra = (-skew(x.segment(3,3)-x.segment(6,3)-x.segment(9,3))*dt).exp();
  Eigen::Matrix3d Rw = (-skew(x.segment(3,3)-x.segment(9,3)+gamma*x.head(3).normalized())*dt).exp();
  
  Fx << Ra,skew(Ra*x.head(3)),-skew(Ra*x.head(3)),-skew(Ra*x.head(3)),
    Eigen::Matrix<double, 3, 3>::Zero(),Eigen::Matrix<double, 3, 3>::Identity(),Eigen::Matrix<double, 3, 3>::Zero(),Eigen::Matrix<double, 3, 3>::Zero(),
    gamma*skew(Rw*x.segment(6,3)),skew(Rw*x.segment(6,3)),Rw,-skew(Rw*x.segment(6,3)),
    Eigen::Matrix<double, 3, 3>::Zero(),Eigen::Matrix<double, 3, 3>::Zero(),Eigen::Matrix<double, 3, 3>::Zero(),Eigen::Matrix<double, 3, 3>::Identity();

  return Fx;

}

void AttEKF::predict(float dt)
{

  Eigen::MatrixXd Fx = dynamics_jacobian(dt);

  P = Fx*P*Fx.transpose() + Q;
  
  x = calc_f(dt);
  
  
}


Eigen::VectorXd AttEKF::calc_h(void)
{

  return x.head(6);

}


Eigen::MatrixXd AttEKF::measurement_jacobian(void)
{

  return Eigen::Matrix<double, 6,12>::Identity();
}



void AttEKF::calc_K(void)
{

  Eigen::MatrixXd Hx = measurement_jacobian();

  K = P*Hx.transpose()*(Hx*P*Hx.transpose()+R).inverse();

}



void AttEKF::correct(Eigen::Vector3d ang, Eigen::Vector3d acc)
{
  Eigen::MatrixXd Hx = measurement_jacobian();

  calc_K();

  Eigen::VectorXd z(6);
  z <<acc,ang;
  
  x = x + K*(z - calc_h());

  //x.head(3) = x.head(3).normalized()*a_mag;
  x.segment(6,3) = ((Eigen::Matrix<double, 3,3>::Identity() - x.head(3).normalized()*x.head(3).normalized().transpose())*x.segment(6,3)).normalized()*w_En_mag;

    
  P = (Eigen::Matrix<double, 12, 12>::Identity() - K*Hx)*P;
  
  
}
