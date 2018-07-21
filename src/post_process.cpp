#include <Eigen/Core>
#include <helper_funcs/helper_funcs.h>
#include <truenorth/att_est.h>
#include <helper_funcs/gyro_data.h>
#include <string>
#include <iostream>
#include <fstream>




int main(int argc, char* argv[])
{

  /**************************************************
   *         CHECK FOR CORRECT ARGUMENTS
   **************************************************/
  if (argc != 2)
  {

    printf("USAGE: post_process CONFIG_FILE.m\n");
    return 1;
    
  }  
  
  /***************************************************
   *
   * LOAD CONFIG FILE
   *
   ***************************************************/

  const config_params params = load_params(argv[1]);
  print_loaded_params(params);


  /***************************************************
   *
   * INITIALIZE ESTIMATOR
   *   
   ***************************************************/
  
  //AttEst att(params.k, params.R0*params.R_align,params.lat);
  AttEst att(params);
  GyroData gyro_data(params.hz);

  printf("***********************************\n");
  printf("    RUNNING ATTITUDE ESTIMATION\n");
  printf("***********************************\n");

  std::ifstream infile(params.i_file.c_str());
  FILE *outfile;
  outfile = fopen(params.o_file.c_str(),"w");

  std::string line;
  double time_start = 0.0;
  bool start = false;
  Eigen::Vector3d att_euler_ang;
  int hours = 0;
  int minutes = 0;

  int cnt = 1;
  
  Eigen::Vector3d rpy_align = rot2rph(params.R_align);
  Eigen::Vector3d rpy_Ro    = rot2rph(params.R0);
  
  fprintf(outfile,"PARAMS,%s,%d,%.10f,%s,%s,%f,%f,%f,%f,%f,%f,%.10f,%.10f,%.10f,%.10f,%.10f,%.10f,%.10f,%.10f,%.10f,%.10f,%.10f,%.10f\n",params.last_mod.c_str(),params.hz,params.lat,params.o_file.c_str(),params.i_file.c_str(),rpy_align(0),rpy_align(1),rpy_align(2),rpy_Ro(0),rpy_Ro(1),rpy_Ro(2),params.K_acc(0,0),params.K_acc(1,1),params.K_acc(2,2),params.K_ang_bias(0,0),params.K_ang_bias(1,1),params.K_ang_bias(2,2),params.K_acc_bias(0,0),params.K_acc_bias(1,1),params.K_acc_bias(2,2),params.K_E_n(0,0),params.K_E_n(1,1),params.K_E_n(2,2));

  while (std::getline(infile, line))
  {

    char msg_type[32];
    int year;
    int month;
    int day;
    int hour;
    int minute;
    float second;

    float rov_time;
    float ros_time;

    Eigen::VectorXi status(6);
    sscanf(line.c_str(),"%s %d/%d/%d %d:%d:%f %f %f %lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%f,%d,%lf,%lf,%d,%d,%d,%d,%d,%d\n",msg_type,&year,&month,&day,&hour,&minute,&second,&rov_time,&ros_time,&gyro_data.ang(0),&gyro_data.ang(1),&gyro_data.ang(2),&gyro_data.acc(0),&gyro_data.acc(1),&gyro_data.acc(2),&gyro_data.mag(0),&gyro_data.mag(1),&gyro_data.mag(2),&gyro_data.temp,&gyro_data.seq_num,&gyro_data.timestamp,&gyro_data.comp_timestamp,&status(0),&status(1),&status(2),&status(3),&status(4),&status(5));

    
    if (!start)
    {

      start = true;
      time_start = gyro_data.timestamp;
      
    }
    float time = gyro_data.timestamp - time_start;


    att.step(gyro_data.ang,9.81*gyro_data.acc,((float) 1)/(float)params.hz);

    att_euler_ang = rot2rph(att.att.R_ni*params.R_align.transpose());
    //att_euler_ang = rot2rph(att.R_ni);


    if (1)//(cnt % (params.hz/10)) == 0)
    {
      fprintf(outfile,"ATT_PRO,%d,%02d,%02d,%02d,%02d,%02f,%f,%f,%f,%f,%.10f,%.10f,%.10f,%.10f,%.10f,%.10f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%d,%d,%d,%d,%d,%d,%d\n",year,month,day,hour,minute,second,gyro_data.timestamp,att_euler_ang(0),att_euler_ang(1),att_euler_ang(2),att.bias.w_b(0),att.bias.w_b(1),att.bias.w_b(2),att.bias.w_E_north(0),att.bias.w_E_north(1),att.bias.w_E_north(2),att.bias.a_b(0),att.bias.a_b(1),att.bias.a_b(2),att.bias.acc_hat(0),att.bias.acc_hat(1),att.bias.acc_hat(2),gyro_data.acc(0),gyro_data.acc(1),gyro_data.acc(2),gyro_data.ang(0),gyro_data.ang(1),gyro_data.ang(2),gyro_data.mag(0),gyro_data.mag(1),gyro_data.mag(2),gyro_data.temp,gyro_data.seq_num,status(0),status(1),status(2),status(3),status(4),status(5));
    }
    if ((((int)time) % (60) == 0) && ((int)time/60 != minutes)) {
      
      hours   = ((int) time)/3600;
      minutes = ((int) time - hours*3600)/60;
      char buffer [256];
      int n = sprintf(buffer,"%02d:%02d:00 OF DATA PROCESSED...",hours,minutes);
      std::cout<<"\r"<<buffer<<std::flush;
    }

    cnt++;

  }
  printf("\n");
  infile.close();
  fclose(outfile);

  return 0;
  
  

}
