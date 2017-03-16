/**
 * @file
 * @date July 2015
 * @author Andrew Spielvogel (andrewspielvogel@gmail.com)
 * @brief Node for publishing sensor data.
 */


#include <ros/ros.h>
#include <truenorth/serial_io.h>
#include <truenorth/gyro_sensor_data.h>
#include <truenorth/helper_funcs.h>
#include <Eigen/Core>
#include <string>
#include <math.h>
#include <truenorth/wqueue.h>
#include <truenorth/gyro_data.h>
#include <truenorth/consumer.h>

#define NODE_RESTART_TIME 1 /**< Seconds without data before restart serial port. */


int main(int argc, char **argv)
{

    // initialize node
    ros::init(argc, argv, "truenorth");

    // must initialize with "~" for param passing
    ros::NodeHandle n("~");

    // initialize publisher
    ros::Publisher chatter = n.advertise<truenorth::gyro_sensor_data>("gyro_data",1000);


    /*
     * Load in params
     */

    // topic publish rate in Hz
    int rate = 10; // default
    n.getParam("rate",rate);
    ros::Rate loop_rate(rate);

    // port name
    std::string port = "/dev/ttyUSB0"; // default
    n.getParam("port",port);

    // log file location
    std::string log_location = "/var/log/KVH/"; // default
    n.getParam("log_loc",log_location);

    // baud rate
    int baud = 921600;
    n.getParam("baud",baud);
    
    // instrument alignment matrix
    std::string instr_align = "1,0,0,0,1,0,0,0,1";  // default
    n.getParam("instr_align",instr_align);
    Eigen::MatrixXd R_align = parse_string(instr_align);
    R_align.resize(3,3);

    // estimation gains
    std::string gains = "1.0,0.005,0.005,0.005,1.0,1.0,0.05,0.005";  // default
    n.getParam("gains",gains);
    
    Eigen::MatrixXd k = parse_string(gains);

    // latitude
    double lat_input = 39.32;
    n.getParam("latitude",lat_input);
    float lat  = (float) lat_input;
    lat = lat*M_PI/180;

    // sample rate
    int hz = 1000; // default
    n.getParam("hz",hz);


   
    /*
     * INITIALIZE SERIAL PORT
     */

    SerialPort serial(k, R_align,log_location.c_str(), hz);
    ConsumerThread* thread = new ConsumerThread(serial.queue,k,R_align,lat,hz);
    thread->start();

    // connect to serial port
    bool connected =  serial.start(port.c_str(),baud);

    if (connected==false)
    {
	ROS_ERROR("port not opened");
	return 0;
    }
    else 
    {
	ROS_INFO("connected to port: %s",port.c_str());
    }


    /*      
     * MAIN LOOP
     */

    // initialize time since data variable
    int cur_time_since_data = 0;
    
    // main loop
    while (ros::ok())
    {

      // initialize data_msg
      truenorth::gyro_sensor_data data_msg;
      ROS_ERROR("%d",serial.queue.size());

      // fill data_msg with data packet
      for (int i=0;i<3;i++)
      {
	data_msg.ang.at(i) = serial.data.ang(i);
	data_msg.acc.at(i) = serial.data.acc(i);
	data_msg.mag.at(i) = serial.data.mag(i);
      }
	
      for (int i=0;i<6;i++)
      {
	data_msg.status.at(i) = serial.data.status.at(i);
      }
	
      data_msg.temp = serial.data.temp;
      data_msg.stamp = serial.data.timestamp;
      data_msg.seq_num = serial.data.seq_num;

      // publish packet
      chatter.publish(data_msg);
  
      ros::spinOnce();
      
      loop_rate.sleep();

    }

    serial.stop();
    return 0;
}
