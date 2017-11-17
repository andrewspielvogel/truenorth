#include <sstream>
#include <fstream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <truenorth/helper_funcs.h>
#include <truenorth/EKF.h>
#include <truenorth/gyro_data.h>
#include <string>
#include <math.h>

int main(int argc, char* argv[])
{

  /***************************************************
   *
   * INPUTS
   *
   ***************************************************/

  // sampling hz
  int hz = 100;//5000;

  // location latitude
  float lat = 39.32*M_PI/180.0;

  // input, output files
  std::string out_file_name = "/home/spiels/log/test.csv";
  std::string in_file_name = "/home/spiels/log/data_test.KVH";

  // alignment (roll,pitch,heading) from vehicle to instrument
  Eigen::Vector3d rpy_align(M_PI,0,M_PI/4.0);
  rpy_align<<0,0,0;

  // initial guess of attitude (in roll, pitch, heading)
  Eigen::Vector3d rpy_R0(M_PI/180.0*1,M_PI/180.0*1,10*M_PI/180.0);
  
  Eigen::Matrix3d R_align = rpy2rot(rpy_align);
  Eigen::Matrix3d R0 = rpy2rot(rpy_R0);

  Eigen::VectorXd x(12);
  x.setZero();
  
  Eigen::Vector3d a_n(0,0,-1);
  Eigen::Vector3d w_E_n(cos(lat)*7.292150/100000.0,0,0);
  Eigen::Vector3d w_E(cos(lat)*7.292150/100000.0,0.0,-sin(lat)*7.292150/100000.0);
  x.head(3) = R_align.transpose()*R0.transpose()*a_n;
  x.segment(6,3) =  R_align.transpose()*R0.transpose()*w_E_n;
  x.segment(3,3) = R_align.transpose()*R0.transpose()*w_E;

  Eigen::VectorXd p(12);
  p << 1,1,1,1,1,1,1,1,1,1,1,1;
  Eigen::VectorXd r(6);

  float w_sig = 6.32*M_PI/180/1000;  // measured 1775, units are rad/sec
  float a_sig = 0.0037;   
  r << a_sig,a_sig,a_sig,w_sig,w_sig,w_sig;

  Eigen::VectorXd q(12);

  q << 1000000,1000000,1000000,100000000,100000000,100000000,.0001,.0001,.0001,.001,.001,.001;

  
  // initialize estimator
  AttEKF att(x,p,r,q/1000.0,lat);
  GyroData gyro_data(hz);
  Eigen::Matrix3d Rni_phins;
  char msg_type[32];
  


  printf("RUNNING ATTITUDE ESTIMATION ON CSV FILE: %s\n",in_file_name.c_str());
  printf("WRITING TO FILE: %s\n",out_file_name.c_str());

  
  std::ifstream infile(in_file_name.c_str());
  FILE *outfile;
  outfile = fopen(out_file_name.c_str(),"w");

  
  std::string line;
  int samp_processed = 0;
  Eigen::Vector3d att_euler_ang;

  Eigen::Vector3d phins_rpy;


  while (std::getline(infile, line))
  {


    sscanf(line.c_str(),"%[^,],%lf,%lf,%lf,%lf,%lf,%lf, %lf,%lf,%lf, %f, %d, %lf,%lf, %*d, %*d, %*d, %*d, %*d, %*d,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf \n",msg_type,&gyro_data.ang(0),&gyro_data.ang(1),&gyro_data.ang(2),&gyro_data.acc(0),&gyro_data.acc(1),&gyro_data.acc(2),&gyro_data.mag(0),&gyro_data.mag(1),&gyro_data.mag(2),&gyro_data.temp,&gyro_data.seq_num,&gyro_data.timestamp,&gyro_data.comp_timestamp,&Rni_phins(0,0),&Rni_phins(0,1),&Rni_phins(0,2),&Rni_phins(1,0),&Rni_phins(1,1),&Rni_phins(1,2),&Rni_phins(2,0),&Rni_phins(2,1),&Rni_phins(2,2),&phins_rpy(0),&phins_rpy(1),&phins_rpy(2));


    Eigen::Matrix3d R_phins = rpy2rot(phins_rpy);



    Eigen::Vector3d q_tilde = R_align.transpose()*R_phins.transpose()*w_E_n;//unskew(R_tilde.log());
    
    fprintf(outfile,"ATT_PRO,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",gyro_data.timestamp,att_euler_ang(0),att_euler_ang(1),att_euler_ang(2),att.x(0),att.x(1),att.x(2),att.x(3),att.x(4),att.x(5),att.x(6),att.x(7),att.x(8),att.x(9),att.x(10),att.x(11),q_tilde(0),q_tilde(1),q_tilde(2));

    att.predict(((float) 1)/(float)hz);
    att.correct(gyro_data.ang,gyro_data.acc);
    
    samp_processed++;

    if ((samp_processed) % (hz*60) == 0) {
      
      int seconds = samp_processed/hz;
      int hours   = seconds/3600;
      int minutes = (seconds - hours*3600)/60;
      seconds = seconds - hours*3600 - minutes*60;
      printf("%02d:%02d:%02d OF DATA PROCESSED\n",hours,minutes,seconds); 

    }

  }
  
  infile.close();
  fclose(outfile);

  return 0;
  
  
  
}
