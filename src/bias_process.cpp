#include <stdlib.h>
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <truenorth/helper_funcs.h>
#include <truenorth/bias_est.h>
#include <string>
#include <math.h>

int main(int argc, char* argv[])
{

  int hz = 5000;
  int rows = hz*60*6;
  int cols = 21;
  float lat = 39.32*M_PI/180;


  Eigen::VectorXd k(4);
  k << 1,0.02,0.02,0.002; //g,w,east_cutoff

  std::string name_out = "/home/spiels/log/processed/processed.csv";
  std::string file = "/home/spiels/log/2017_1_25_9_53.KVH";


  printf("LOADING CSV FILE: %s\n",file.c_str());

  Eigen::MatrixXd data = readCSV(file,rows,cols);

  printf("CSV FILE LOADED: %s\n",file.c_str());

  printf("RUNNING ATTITUDE ESTIMATION\n");

  BiasEst bias(k,lat);

  Eigen::MatrixXd trph(10,rows-1);

  Eigen::Matrix3d Rni;
  Rni << 1,0,0,0,-1,0,0,0,-1;

  for (int i=1; i<rows; i++) {

    float seq_diff = data(i,10)-data(i-1,10);
    if (seq_diff < 0)
    {
	
      seq_diff += 128;

    }


    bias.step(Rni,data.block<1,3>(i,0).transpose(),data.block<1,3>(i,3).transpose(),data(i,11)-data(0,11));   
   
    trph(0,i-1) = data(i,11)-data(0,11);
    trph.block<3,1>(1,i-1) = bias.w_b;
    trph.block<3,1>(4,i-1) = bias.a_b;
    trph.block<3,1>(7,i-1) = bias.z;

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
