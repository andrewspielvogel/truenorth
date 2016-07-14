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

  int rows = 5000*60*7;
  int cols = 21;
  float lat = 39.32*180/M_PI;
  Eigen::Matrix3d R_align;
  R_align << 1,0,0,0,-1,0,0,0,-1;
  Eigen::VectorXd k(4);
  k << 1,.001,1,.01; //a,e,g,w

  std::string name_out = "test.csv";

  std::string file = "/home/spiels/log/KVH/static_run3/2016_7_13_15_23.KVH";

  printf("LOADING CSV FILE: %s\n",file.c_str());

  Eigen::MatrixXd data = readCSV(file,rows,cols);

  printf("CSV FILE LOADED: %s\n",file.c_str());

  Eigen::Matrix3d R0;
  R0 = get_R_sn(lat,0)*R_align;

  AttEst att(k,R0);

  Eigen::MatrixXd trph(4,rows-1);

  for (int i=1; i<rows; i++) {

    att.step(data.block<1,3>(i,0).transpose(),data.block<1,3>(i,3).transpose(),data(i,11)-data(0,11),data(i,11)-data(i-1,11));
   
    trph(0,i-1) = data(i,11);
    trph.block<3,1>(1,i-1) = rot2rph(att.R_ni*R_align);

    if ((i+1) % (5000*60) == 0){

      std::cout<<trph.block<3,1>(0,i-1)*180/M_PI<<std::endl<<std::endl;

    }

  }

  Eigen::IOFormat CSVFormat(Eigen::FullPrecision, Eigen::DontAlignCols, ", ", "\n");
  std::ofstream ofile(name_out.c_str());
  ofile << trph.format(CSVFormat);
  ofile.close();
  

}
