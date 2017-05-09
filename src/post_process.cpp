#include <stdlib.h>
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <truenorth/helper_funcs.h>
#include <truenorth/att_est.h>
#include <string>
#include <math.h>

int main(int argc, char* argv[])
{

  int hz = 5000;
  int rows = hz*60*5;
  int cols = 28;
  float lat = 39.32*M_PI/180;
  Eigen::Matrix3d R_align;
  R_align << 1,0,0,0,-1,0,0,0,-1;
  //R_align << 1,0,0,0,cos(M_PI/4),-sin(M_PI/4),0,sin(M_PI/4),cos(M_PI/4);

  Eigen::Vector3d w_err(1,1,1);
  w_err = -w_err*10*M_PI/180;
  Eigen::Matrix3d R_err = skew(w_err).exp();

  Eigen::VectorXd k(3);
  k << 0.1,0.1,1000; //g,w,east_cutoff

  std::string name_out = "/home/spiels/log/data.csv";
  std::string file = "/home/spiels/log/data2.KVH";


  printf("LOADING CSV FILE: %s\n",file.c_str());

  Eigen::MatrixXd data = readCSV(file,rows,cols);

  printf("CSV FILE LOADED: %s\n",file.c_str());

  printf("RUNNING ATTITUDE ESTIMATION\n");

  AttEst att(k, R_align*R_err,lat,hz);

  Eigen::MatrixXd trph(10,rows-1);

  Eigen::Vector3d bias_offset_a(0.003,-0.0044,0.0018);
  Eigen::Vector3d bias_offset_w(0.0546/10000.0,0.2530/10000.0,0.0057/10000.0);
  Eigen::Matrix3d Rni;

  for (int i=1; i<rows; i++) {

    float seq_diff = data(i,10)-data(i-1,10);
    if (seq_diff < 0)
    {
	
      seq_diff += 128;

    }

    Rni << data(i-1,19),data(i-1,20),data(i-1,21),data(i-1,22),data(i-1,23),data(i-1,24),data(i-1,25),data(i-1,26),data(i-1,27);

    att.step(data.block<1,3>(i,0).transpose()-bias_offset_w,data.block<1,3>(i,3).transpose()-bias_offset_a,((float) 1)/(float)hz);
    
    trph(0,i-1) = data(i,11)-data(0,11);
    trph.block<3,1>(1,i-1) = rot2rph(att.R_ni*R_align);
    trph.block<3,1>(4,i-1) = rot2rph(Rni*R_align);
    trph.block<3,1>(7,i-1) = att.east_est_n_;//att.R_ni.transpose()*att.a_n;
    



    if ((i) % (hz*30) == 0) {
      
      int seconds = i/hz;
      int hours   = seconds/3600;
      int minutes = (seconds - hours*3600)/60;
      seconds = seconds - hours*3600 - minutes*60;
      printf("%02d:%02d:%02d OF DATA PROCESSED\n",hours,minutes,seconds); 

    }

  }

  printf("WRITING TO FILE: %s\n",name_out.c_str());

  Eigen::IOFormat CSVFormat(Eigen::FullPrecision, Eigen::DontAlignCols, ", ", "\n");
  std::ofstream ofile(name_out.c_str());

  ofile << trph.format(CSVFormat);
  ofile.close();

}
