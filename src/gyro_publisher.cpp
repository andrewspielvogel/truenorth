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
#include <truenorth/bias_consumer.h>

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

    BiasConsumerThread* bias_thread = new BiasConsumerThread(serial.bias_queue,k.block<4,1>(3,0),lat,hz);
    
    thread->start();
    bias_thread->start();
    
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
      if (serial.queue.size()>100)
      {
	ROS_ERROR("Att queue exceeds 100 - Size: :%d",serial.queue.size());
      }
      if (serial.bias_queue.size()>100)
      {
	ROS_ERROR("Bias Estimation queue exceeds 100 - Size: :%d",serial.bias_queue.size());
      }
      
      // fill data_msg with data packet
      for (int i=0;i<3;i++)
      {
	data_msg.imu.ang.at(i) = serial.data.ang(i);
	data_msg.imu.acc.at(i) = serial.data.acc(i);
	data_msg.imu.mag.at(i) = serial.data.mag(i);
	data_msg.att.at(i) = 180*rot2rph((thread->att.R_ni)*R_align)(i)/M_PI;
	data_msg.bias.ang.at(i) = bias_thread->bias.w_b(i);
	data_msg.bias.acc.at(i) = bias_thread->bias.a_b(i);
	data_msg.bias.z.at(i) = bias_thread->bias.z(i);
      }

      for (int i=0;i<6;i++)
      {
	data_msg.imu.status.at(i) = serial.data.status.at(i);
      }
	
      data_msg.imu.temp = serial.data.temp;
      data_msg.imu.stamp = serial.data.timestamp;
      data_msg.imu.seq_num = serial.data.seq_num;

      // publish packet
      chatter.publish(data_msg);
  
      ros::spinOnce();
      
      loop_rate.sleep();

    }

    serial.stop();
    return 0;
}
