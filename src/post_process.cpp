#include <sstream>
#include <fstream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <truenorth/helper_funcs.h>
#include <truenorth/att_est.h>
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
  int hz = 5000;

  // location latitude
  float lat = 39.32*M_PI/180.0;

  // input, output files
  std::string out_file_name = "/home/spiels/log/test.csv";
  std::string in_file_name = "/home/spiels/log/ICRA2018/run1/2017_8_16_11_31.KVH";
  //std::string in_file_name = "/home/spiels/log/data_test.KVH";//RSS/2017_11_6_12_37.KVH";//ICRA2018/run1/2017_8_16_11_31.KVH";

  // alignment (roll,pitch,heading) from vehicle to instrument
  Eigen::Vector3d rpy_align(M_PI,0,M_PI/4.0);
  //rpy_align<<0,0,0;

  // initial guess of attitude (in roll, pitch, heading)
  Eigen::Vector3d rpy_R0(M_PI/180.0*0,M_PI/180.0,5*M_PI/180.0);

  // estimator gains
  Eigen::VectorXd k(6);
  //k << 1,100,0.05,0.0001,0.0005,1; //g,w,kf
  k << 1,150,.005,0.00001,0.00005,0.25;



  Eigen::Matrix3d R_align = rpy2rot(rpy_align);
  Eigen::Matrix3d R0 = rpy2rot(rpy_R0);

  // initialize estimator
  AttEst att(k, R0*R_align,lat,hz);
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

  Eigen::Matrix3d R_en = get_R_en(lat);
  double earthrate = 7.292150/100000.0;
  Eigen::Vector3d w_E(0,0,earthrate);
  Eigen::Vector3d w_E_n = R_en.transpose()*w_E;
  w_E_n(2) = 0.0;

  while (std::getline(infile, line))
  {


    sscanf(line.c_str(),"%[^,],%lf,%lf,%lf,%lf,%lf,%lf, %lf,%lf,%lf, %f, %d, %lf,%lf, %*d, %*d, %*d, %*d, %*d, %*d,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf \n",msg_type,&gyro_data.ang(0),&gyro_data.ang(1),&gyro_data.ang(2),&gyro_data.acc(0),&gyro_data.acc(1),&gyro_data.acc(2),&gyro_data.mag(0),&gyro_data.mag(1),&gyro_data.mag(2),&gyro_data.temp,&gyro_data.seq_num,&gyro_data.timestamp,&gyro_data.comp_timestamp,&Rni_phins(0,0),&Rni_phins(0,1),&Rni_phins(0,2),&Rni_phins(1,0),&Rni_phins(1,1),&Rni_phins(1,2),&Rni_phins(2,0),&Rni_phins(2,1),&Rni_phins(2,2),&phins_rpy(0),&phins_rpy(1),&phins_rpy(2));

    //sscanf(line.c_str(),"%[^,],%lf,%lf,%lf,%lf,%lf,%lf, %lf,%lf,%lf, %f, %d, %lf, %*d, %*d, %*d, %*d, %*d, %*d \n",msg_type,&gyro_data.ang(0),&gyro_data.ang(1),&gyro_data.ang(2),&gyro_data.acc(0),&gyro_data.acc(1),&gyro_data.acc(2),&gyro_data.mag(0),&gyro_data.mag(1),&gyro_data.mag(2),&gyro_data.temp,&gyro_data.seq_num,&gyro_data.timestamp);

    Eigen::Matrix3d R_phins = rpy2rot(phins_rpy);

    att.step(gyro_data.ang,gyro_data.acc,((float) 1)/(float)hz);

    att_euler_ang = rot2rph(att.R_ni*R_align.transpose());

    Eigen::Vector3d e(0,1,0);
    Eigen::Matrix3d R_tilde = R_phins.transpose()*att.R_ni*R_align.transpose();
    Eigen::Vector3d q_tilde = R_align.transpose()*R_phins.transpose()*w_E_n;//unskew(R_tilde.log());
    
    fprintf(outfile,"ATT_PRO,%f,%f,%f,%f,%f,%f,%f,%.10f,%.10f,%.10f,%.10f,%.10f,%.10f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",gyro_data.timestamp,att_euler_ang(0),att_euler_ang(1),att_euler_ang(2),phins_rpy(0),phins_rpy(1),phins_rpy(2),att.w_b(0),att.w_b(1),att.w_b(2),att.w_E_north(0),att.w_E_north(1),att.w_E_north(2),att.a_b(0),att.a_b(1),att.a_b(2),q_tilde(0),q_tilde(1),q_tilde(2),att.acc_hat(0),att.acc_hat(1),att.acc_hat(2));

    //fprintf(outfile,"ATT_PRO,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",gyro_data.timestamp,att_euler_ang(0),att_euler_ang(1),att_euler_ang(2),att.g_error_(0),att.g_error_(1),att.g_error_(2),att.h_error_(0),att.h_error_(1),att.h_error_(2),att.east_est_n(0),att.east_est_n(1),att.east_est_n(2));
    
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
