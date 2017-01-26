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

  int hz = 1000;
  int rows = hz*60*29;
  int cols = 21;
  float lat = 39.32*M_PI/180;
  Eigen::Matrix3d R_align;
  R_align << 1,0,0,0,1,0,0,0,1;

  Eigen::Vector3d w_err(1,1,1);
  w_err = w_err*5*M_PI/180;
  Eigen::Matrix3d R_err = skew(w_err).exp();

  Eigen::VectorXd k(3);
  k << 1,.03,.01; //g,w,east_cutoff

  std::string name_out = "/home/spiels/processed.csv";
  std::string file = "/home/spiels/data.KVH";


  printf("LOADING CSV FILE: %s\n",file.c_str());

  Eigen::MatrixXd data = readCSV(file,rows,cols);

  printf("CSV FILE LOADED: %s\n",file.c_str());

  printf("RUNNING ATTITUDE ESTIMATION\n");

  AttEst att(k, R_align*R_err,lat,hz);

  Eigen::MatrixXd trph(10,rows-1);

  //Eigen::Vector3d bias_offset(0.059109,0.138986,-0.405373);
  //Eigen::Vector3d bias_offset(-0.044764,0.159871,-0.459614);
  Eigen::Vector3d bias_offset(0.0,0.0,0.0);

  for (int i=1; i<rows; i++) {

    float seq_diff = data(i,10)-data(i-1,10);
    if (seq_diff < 0)
    {
	
      seq_diff += 128;

    }


    att.step(data.block<1,3>(i,0).transpose(),data.block<1,3>(i,3).transpose(),data(i,11)-data(0,11),((float) 1)/(float)hz);   
    //att.step(data.block<1,3>(i,0).transpose()-bias_offset/10000,data.block<1,3>(i,3).transpose(),((float) i)/(float)hz,((float) 1)/(float)hz);   


    trph(0,i-1) = data(i,11)-data(0,11);
    trph.block<3,1>(1,i-1) = rot2rph(att.R_ni*R_align);
    trph.block<3,1>(4,i-1) = rot2rph(att.Rb_);
    trph.block<3,1>(7,i-1) = att.east_est_n_;

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
