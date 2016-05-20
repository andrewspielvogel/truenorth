/*
 * data_2_csv.cpp
 * KVH 1775 rosbag parser to csv file
 *
 * created July 2015
 * Andrew Spielvogel
 * andrewspielvogel@gmail.com
 */

#include "ros/ros.h"

#include "kvh_1775/gyro_sensor_data.h"
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <iostream>
#include <fstream>

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

int main(int argc, char **argv)
{

  if (argc != 3) {
    ROS_ERROR("declare input and output files");
    return 0;
  }

    char *in = argv[1];
    char *out= argv[2];
    std::ofstream myfile;
    myfile.open (out);
    rosbag::Bag bag;
    bag.open(in, rosbag::bagmode::Read);
    rosbag::View view(bag);
    foreach(rosbag::MessageInstance const m, view)
    {
	kvh_1775::gyro_sensor_data::ConstPtr s = m.instantiate<kvh_1775::gyro_sensor_data>();
	if (s != NULL){
	    myfile << s->ang[0] << "," << s->ang[1] << "," << s->ang[2] << ","
	    	   << s->acc[0] << "," << s->acc[1] << "," << s->acc[2] << ","
	    	   << s->mag[0] << "," << s->mag[1] << "," << s->mag[2] << ","
	    	   << (int) s->status[0] << "," << (int) s->status[1] << "," << (int) s->status[2] << ","
	    	   << (int) s->status[3] << "," << (int) s->status[4] << "," << (int) s->status[5] << ","
	    	   << s->temp << "," << s->stamp << "," << (int) s->seq_num << ","
		   << s->acc_est[0] << "," << s->acc_est[1] << "," << s->acc_est[2] << ","
		   << s->bias_ang[0] << "," << s->bias_ang[1] << "," << s->bias_ang[2] << ","
		   << s->bias_acc[0] << "," << s->bias_acc[1] << "," << s->bias_acc[2] << ","
		   << s->bias_z[0] << "," << s->bias_z[1] << "," << s->bias_z[2] << ","
	           << s->att[0] << "," << s->att[1] << "," << s->att[2] << "\n";
	}

    }
    myfile.close();
    bag.close();
}

