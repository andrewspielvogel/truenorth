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

  int rows = 5000*60*8.5;
  int cols = 21;
  float lat = 39.32*180/M_PI;
  Eigen::Matrix3d R_align;
  R_align << 1,0,0,0,-1,0,0,0,-1;
  Eigen::VectorXd k(4);
  k << 1,.01,1,.01; //a,e,g,w

  std::string name_out = "test.csv";
  std::string file = "/home/spiels/log/KVH/static_run3/2016_7_13_15_23.KVH";



  printf("LOADING CSV FILE: %s\n",file.c_str());

  Eigen::MatrixXd data = readCSV(file,rows,cols);

  printf("CSV FILE LOADED: %s\n",file.c_str());

  printf("RUNNING ATTITUDE ESTIMATION\n");


  AttEst att(k,R_align);

  Eigen::MatrixXd trph(7,rows-1);

  for (int i=1; i<rows; i++) {

    att.step(data.block<1,3>(i,0).transpose(),data.block<1,3>(i,3).transpose(),data(i,11)-data(0,11),data(i,11)-data(i-1,11));
   
    trph(0,i-1) = data(i,11);
    trph.block<3,1>(1,i-1) = rot2rph(att.R_ni*R_align);
    trph.block<3,1>(4,i-1) = rot2rph(att.Rb_);

  }

  Eigen::IOFormat CSVFormat(Eigen::FullPrecision, Eigen::DontAlignCols, ", ", "\n");
  std::ofstream ofile(name_out.c_str());

  ofile << trph.format(CSVFormat);
  ofile.close();
  
  std::cout<<att.Rb_<<std::endl;
std::cout<<att.R_ni<<std::endl;

}
