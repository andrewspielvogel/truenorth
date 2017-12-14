#include <sstream>
#include <fstream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <truenorth/helper_funcs.h>
#include <truenorth/att_est.h>
#include <truenorth/gyro_data.h>
#include <string>
#include <math.h>

struct config_params {
  
  int hz;
  float lat;
  std::string o_file;
  std::string i_file;
  std::string last_mod;
  Eigen::Vector3d rpy_align;
  Eigen::Vector3d rpy_Ro;
  Eigen::VectorXd k;
  
};

config_params load_params(char* config_file)
{

  printf("LOADING CONFIG FILE: %s\n",config_file);
  
  config_params params;
  std::ifstream infile(config_file);
  std::string line;

  /**********************************
   *  LOAD IN LINE FROM CONFIG FILE
   **********************************/
  while (std::getline(infile, line))
  {

    char field[128] = "";
    char data[128]  = "";

    /*********************************
     *          PARSE LINE
     *********************************/
    sscanf(line.c_str(),"%s = %s",field,data);
    
    if((std::string(field))=="hz")
    {

      sscanf(data,"%d",&params.hz);

    }
    else if((std::string(field))=="lat")
    {

      sscanf(data,"%f",&params.lat);
      params.lat = params.lat*M_PI/180.0;

    }
    else if((std::string(field))=="o_file")
    {

      char str[128];
      sscanf(data,"%s",str);
      params.o_file = std::string(str);
      params.o_file.erase(remove(params.o_file.begin(),params.o_file.end(), '\"' ),params.o_file.end());

    }
    else if((std::string(field))=="i_file")
    {
	
      char str[128];
      sscanf(data,"%s",str);
      params.i_file = std::string(str);
      params.i_file.erase(remove(params.i_file.begin(),params.i_file.end(), '\"' ),params.i_file.end());
      
    }
    else if((std::string(field))=="rpy_align")
    {

      sscanf(data,"[%lf,%lf,%lf]",&params.rpy_align(0),&params.rpy_align(1),&params.rpy_align(2));

    }
    else if((std::string(field))=="rpy_Ro")
    {

      sscanf(data,"[%lf,%lf,%lf]",&params.rpy_Ro(0),&params.rpy_Ro(1),&params.rpy_Ro(2));

    }
    else if((std::string(field))=="k")
    {

      Eigen::VectorXd k(6);
      params.k = k;
      sscanf(data,"[%lf,%lf,%lf,%lf,%lf,%lf]",&params.k(0),&params.k(1),&params.k(2),&params.k(3),&params.k(4),&params.k(5));

    }
    else if ((std::string(field))=="last_mod")
    {

      char str[128];
      sscanf(data,"%s",str);
      params.last_mod = std::string(str);
      params.last_mod.erase(remove(params.last_mod.begin(),params.last_mod.end(), '\"' ),params.last_mod.end());
	
    }
    
  }

  return params;

}

void print_loaded_params(config_params params)
{

  printf("***********************************\n");
  printf("           LOADED PARAMS \n");
  printf("***********************************\n");
  printf(" last_mod: %s\n",params.last_mod.c_str());
  printf("       hz: %d (s^-1)\n",params.hz);
  printf("      lat: %.10f (rad)\n",params.lat);
  printf("   o_file: %s\n",params.o_file.c_str());
  printf("   i_file: %s\n",params.i_file.c_str());
  printf("rpy_align: [%lf,%lf,%lf] (rad)\n",
	 params.rpy_align(0),params.rpy_align(1),params.rpy_align(2));
  printf("   rpy_Ro: [%lf,%lf,%lf] (rad)\n",
	 params.rpy_Ro(0),params.rpy_Ro(1),params.rpy_Ro(2));
  printf("        k: [%lf,%lf,%lf,%lf,%lf,%lf]\n",
	 params.k(0),params.k(1),params.k(2),params.k(3),params.k(4),params.k(5));

}


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
   
  const Eigen::Matrix3d R_align = rpy2rot(params.rpy_align);
  const Eigen::Matrix3d R0 = rpy2rot(params.rpy_Ro);


  /***************************************************
   *
   * INITIALIZE ESTIMATOR
   *   
   ***************************************************/
  
  AttEst att(params.k, R0*R_align,params.lat,params.hz);
  GyroData gyro_data(params.hz);
  Eigen::Matrix3d Rni_phins;
  char msg_type[32];

  printf("***********************************\n");
  printf("    RUNNING ATTITUDE ESTIMATION\n");
  printf("***********************************\n");

  std::ifstream infile(params.i_file.c_str());
  FILE *outfile;
  outfile = fopen(params.o_file.c_str(),"w");

  std::string line;
  float time_start = 0.0;
  bool start = false;
  Eigen::Vector3d att_euler_ang;
  Eigen::Vector3d phins_rpy;
  int hours = 0;
  int minutes = 0;

  int cnt = 1;
  

  fprintf(outfile,"PARAMS,%s,%d,%.10f,%s,%s,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",params.last_mod.c_str(),params.hz,params.lat,params.o_file.c_str(),params.i_file.c_str(),params.rpy_align(0),params.rpy_align(1),params.rpy_align(2),params.rpy_Ro(0),params.rpy_Ro(1),params.rpy_Ro(2),params.k(0),params.k(1),params.k(2),params.k(3),params.k(4),params.k(5));

  while (std::getline(infile, line))
  {

    sscanf(line.c_str(),"%[^,],%lf,%lf,%lf,%lf,%lf,%lf, %lf,%lf,%lf, %f, %d, %lf,%lf, %*d, %*d, %*d, %*d, %*d, %*d,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf \n",msg_type,&gyro_data.ang(0),&gyro_data.ang(1),&gyro_data.ang(2),&gyro_data.acc(0),&gyro_data.acc(1),&gyro_data.acc(2),&gyro_data.mag(0),&gyro_data.mag(1),&gyro_data.mag(2),&gyro_data.temp,&gyro_data.seq_num,&gyro_data.timestamp,&gyro_data.comp_timestamp,&Rni_phins(0,0),&Rni_phins(0,1),&Rni_phins(0,2),&Rni_phins(1,0),&Rni_phins(1,1),&Rni_phins(1,2),&Rni_phins(2,0),&Rni_phins(2,1),&Rni_phins(2,2),&phins_rpy(0),&phins_rpy(1),&phins_rpy(2));

    if (!start)
    {

      start = true;
      time_start = gyro_data.timestamp;
      
    }
    float time = gyro_data.timestamp - time_start;
    const Eigen::Matrix3d R_phins = rpy2rot(phins_rpy);

    att.step(gyro_data.ang,gyro_data.acc,gyro_data.mag,((float) 1)/(float)params.hz,gyro_data.timestamp);

    //att_euler_ang = rot2rph(att.R_ni*R_align.transpose());
    att_euler_ang = rot2rph(att.R_ni);

    //const Eigen::Matrix3d R_tilde = R_phins.transpose()*att.R_ni*R_align.transpose();
    const Eigen::Matrix3d R_tilde = R_phins.transpose()*att.R_ni;
    const Eigen::Vector3d q_tilde = rot2rph(R_tilde);

    if ((cnt % 1000) == 0)
    {
      fprintf(outfile,"ATT_PRO,%f,%f,%f,%f,%f,%f,%f,%.10f,%.10f,%.10f,%.10f,%.10f,%.10f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",gyro_data.timestamp,att_euler_ang(0),att_euler_ang(1),att_euler_ang(2),phins_rpy(0),phins_rpy(1),phins_rpy(2),att.w_b(0),att.w_b(1),att.w_b(2),att.w_E_north(0),att.w_E_north(1),att.w_E_north(2),att.a_b(0),att.a_b(1),att.a_b(2),q_tilde(0),q_tilde(1),q_tilde(2),att.acc_hat(0),att.acc_hat(1),att.acc_hat(2),gyro_data.acc(0),gyro_data.acc(1),gyro_data.acc(2),gyro_data.ang(0),gyro_data.ang(1),gyro_data.ang(2),gyro_data.mag(0),gyro_data.mag(1),gyro_data.mag(2));
    }
    if ((((int)time) % (60) == 0) && ((int)time/60 != minutes)) {
      
      hours   = ((int) time)/3600;
      minutes = ((int) time - hours*3600)/60;
      printf("%02d:%02d:00 OF DATA PROCESSED\n",hours,minutes); 

    }

    cnt++;

  }
  
  infile.close();
  fclose(outfile);

  return 0;
  
  

}
