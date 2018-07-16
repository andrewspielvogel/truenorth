#include <sstream>
#include <fstream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <helper_funcs/helper_funcs.h>
#include <string>
#include <iostream>

int main(int argc, char* argv[])
{

  /**************************************************
   *         CHECK FOR CORRECT ARGUMENTS
   **************************************************/
  if (argc != 3)
    {

      printf("USAGE: post_process log_file.MSA3\n");
      return 1;
    
    }  
  
 


  std::ifstream infile(argv[1]);
  std::string line;
  FILE *outfile;
  outfile = fopen(argv[2],"w");



  while (std::getline(infile, line))
    {

      Eigen::Vector3f att(0,0,0);
      Eigen::Vector3f ang(0,0,0);
      Eigen::Vector3f acc(0,0,0);
      Eigen::Vector3f mag(0,0,0);
      Eigen::VectorXf q(4);
      
      int timer_ticks;
      int num_bad;
      int num_good;

      Eigen::Matrix3d R;

      int year;
      int month;
      int day;
      int hour;
      int minute;
      float second;

      char msg_type[32];
      
      

      sscanf(line.c_str(),"%*s %d/%d/%d %d:%d:%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %d %d %d %lf %lf %lf %lf %lf %lf %lf %lf %lf %*s \n",&year,&month,&day,&hour,&minute,&second,&att(0),&att(1),&att(2),&ang(0),&ang(1),&ang(2),&acc(0),&acc(1),&acc(2),&q(0),&q(1),&q(2),&q(3),&mag(0),&mag(1),&mag(2),&timer_ticks,&num_bad,&num_good,&R(0,0),&R(0,1),&R(0,2),&R(1,0),&R(1,1),&R(1,2),&R(2,0),&R(2,1),&R(2,2));
      
      float time = ((float) timer_ticks)/62500.0;

      Eigen::Vector3d rph = rot2rph(R);
      
      fprintf(outfile,"MS3,%f,%f,%f,%f,%f,%f,%f,%f,%f,0,%d,%f,%f,1,1,1,1,1,1,%f,%f,%f\n",ang(0),ang(1),ang(2),acc(0),acc(1),acc(2),mag(0),mag(1),mag(2),num_good+num_bad,time,time,rph(0),rph(1),rph(2));
 
      
  
    } 
  infile.close();
  fclose(outfile);

  return 0;
}
