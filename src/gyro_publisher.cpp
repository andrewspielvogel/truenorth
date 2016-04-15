/*
 * gyro_publisher.cpp
 * node for publishing sensor data
 *
 * created July 2015
 * Andrew Spielvogel
 * andrewspielvogel@gmail.com
 */


#include "ros/ros.h"
#include "std_msgs/String.h"
#include <kvh_1775/serial_io.h>
#include "kvh_1775/gyro_sensor_data.h"

#include <sstream>


int main(int argc, char **argv)
{

    // initialize
    ros::init(argc, argv, "kvh_1775");

    ros::NodeHandle n("~");

    // initialize publisher
    ros::Publisher chatter = n.advertise<kvh_1775::gyro_sensor_data>("gyro_data",1000);

    // rate in Hz
    ros::Rate loop_rate(1000);

    // port name
    std::string name = "/dev/ttyUSB0";
    n.getParam("port",name);
    
    // initialize serial port and data
    Eigen::Vector3d init_sig(0,0,0);
    std::vector<bool> init_stat(6,false);
    SerialPort serial;
    serial.data.set_values(init_sig, init_sig, 0.0, init_stat, 500);

    // connect to serial port
    bool rv;
    rv = serial.start(name.c_str(),921600);
    if (rv==false)
    {
	ROS_ERROR("port not opened");
	return 0;
    }
    else 
    {
	ROS_INFO("connected to port: %s",name.c_str());
    }



    while (ros::ok())
    {

	// initialize data_msg
	kvh_1775::gyro_sensor_data data_msg;

	// fill data_msg with sensor packet
	for (int i=0;i<3;i++)
	{
	    data_msg.ang.at(i) = serial.data.ang(i);
	    data_msg.acc.at(i) = serial.data.acc(i);
	    data_msg.mag.at(i) = serial.data.mag(i);
	    data_msg.acc_est.at(i) = serial.data.acc_est(i);
	    data_msg.bias_acc.at(i) = serial.data.bias_acc(i);
	    data_msg.bias_ang.at(i) = serial.data.bias_ang(i);
	    data_msg.bias_z.at(i) = serial.data.bias_z(i);
	}
	
	for (int i=0;i<6;i++)
	{
	    data_msg.status.at(i) = serial.data.status.at(i);
	}
	
	data_msg.temp = serial.data.temp;
	data_msg.stamp = ros::Time::now();
	data_msg.t = (double)data_msg.stamp.sec+(double)data_msg.stamp.nsec/1000000000.0;
	data_msg.seq_num = serial.data.seq_num;

	// publish packet
	chatter.publish(data_msg);
    
	ros::spinOnce();

	loop_rate.sleep();

    }

    serial.stop();
    return 0;
}
