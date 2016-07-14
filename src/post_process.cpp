#include <stdlib.h>
#include <iostream>
#include <Eigen/Dense>
#include <truenorth/helper_funcs.h>
#include <truenorth/att_est.h>
#include <string>

int main(int argc, char* argv[])
{

  std::string file = "/home/spiels/log/KVH/static_run3/2016_7_13_15_23.KVH";

  Eigen::MatrixXd data = readCSV(file,5,21);

  Eigen::VectorXd k(4);
  k << 1,.0001,1,.005; //a,e,g,q
  AttEst att(k,Eigen::Matrix3d::Identity(3,3));

}
