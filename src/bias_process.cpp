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
  int rows = hz*60*3;
  int cols = 28;//12;
  float lat = 39.32*M_PI/180;


  Eigen::VectorXd k(4);
  //k << .1,.0003,0.0,0.0;
  k<<10,0.05,0.0,0.0;
  std::string name_out = "/home/spiels/log/processedbias.csv";
  std::string file = "/home/spiels/log/data2.KVH";


  printf("LOADING CSV FILE: %s\n",file.c_str());

  Eigen::MatrixXd data = readCSV(file,rows,cols);

  printf("CSV FILE LOADED: %s\n",file.c_str());

  printf("RUNNING BIAS ESTIMATION\n");

  BiasEst bias(k,lat);

  Eigen::MatrixXd trph(10,rows-1);

  Eigen::Matrix3d Rni;
  Rni <<1,0,0,0,-1,0,0,0,-1;
  Eigen::Matrix3d Ralign;
  Ralign << 1,0,0,0,-1,0,0,0,-1;
  Eigen::Vector3d w_err(0,0,3*M_PI/180);
  Eigen::Matrix3d R_err = skew(w_err*0).exp();

  for (int i=1; i<rows; i++) {

    float seq_diff = data(i,10)-data(i-1,10);
    if (seq_diff < 0)
    {
	
      seq_diff += 128;

    }


    Rni << data(i,19),data(i,20),data(i,21),data(i,22),data(i,23),data(i,24),data(i,25),data(i,26),data(i,27);


    bias.step(Rni*Ralign.transpose()*R_err,data.block<1,3>(i,0).transpose(),data.block<1,3>(i,3).transpose(),1.0/hz);

   
    trph(0,i-1) = data(i,11)-data(0,11);
    trph.block<3,1>(1,i-1) = rot2rph(Rni*Ralign);
    trph.block<3,1>(4,i-1) = data.block<1,3>(i,0).transpose();
    trph.block<3,1>(7,i-1) = data.block<1,3>(i,3).transpose();

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
