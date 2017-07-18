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
  
  int hz = 5000;
  float lat = 39.32*M_PI/180;

  
  std::string out_file_name = "/home/spiels/log/data.csv";
  std::string in_file_name = "/home/spiels/log/test/data.KVH";

  Eigen::Vector3d rpy(M_PI,0,M_PI/4.0);

  Eigen::VectorXd k(3);
  k << 0.005,0.00005,10; //g,w,east_cutoff

  Eigen::Matrix3d R_align = rpy2rot(rpy);

  Eigen::Vector3d w_err(.1,.1,1);
  w_err = w_err*1.5*M_PI/180;
  Eigen::Matrix3d R_err = skew(w_err).exp();




  AttEst att(k, R_align*R_err,lat,hz);
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

  Eigen::Vector3d w_b(5.1/1000000.0,8.5/1000000.0,-2.31/100000.0);
  Eigen::Vector3d a_b(0.003,-0.005,0.002);
  w_b = w_b;
  
  
  while (std::getline(infile, line))
  {


    sscanf(line.c_str(),"%[^,],%lf,%lf,%lf,%lf,%lf,%lf, %lf,%lf,%lf, %f, %d, %lf,%lf, %*d, %*d, %*d, %*d, %*d, %*d,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf \n",msg_type,&gyro_data.ang(0),&gyro_data.ang(1),&gyro_data.ang(2),&gyro_data.acc(0),&gyro_data.acc(1),&gyro_data.acc(2),&gyro_data.mag(0),&gyro_data.mag(1),&gyro_data.mag(2),&gyro_data.temp,&gyro_data.seq_num,&gyro_data.timestamp,&gyro_data.comp_timestamp,&Rni_phins(0,0),&Rni_phins(0,1),&Rni_phins(0,2),&Rni_phins(1,0),&Rni_phins(1,1),&Rni_phins(1,2),&Rni_phins(2,0),&Rni_phins(2,1),&Rni_phins(2,2));

    Eigen::Vector3d phins_rpy = rot2rph(Rni_phins);

    att.step(gyro_data.ang - w_b,gyro_data.acc,((float) 1)/(float)hz);

    att_euler_ang = rot2rph(att.R_ni*R_align.transpose());
    
    fprintf(outfile,"ATT_PRO,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",gyro_data.timestamp,att_euler_ang(0),att_euler_ang(1),att_euler_ang(2),phins_rpy(0),phins_rpy(1),phins_rpy(2),gyro_data.acc(0),gyro_data.acc(1),gyro_data.acc(2),att.h_error_(0),att.h_error_(1),att.h_error_(2));
    
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
