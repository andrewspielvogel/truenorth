/**
 * @file
 * @date July 2015
 * @author Andrew Spielvogel (andrewspielvogel@gmail.com)
 * @brief Node for publishing sensor data.
 */


#include <ros/ros.h>
#include <truenorth/serial_io.h>
#include <truenorth/gyro_sensor_data.h>
#include <phins/phins_msg.h>
#include <truenorth/helper_funcs.h>
#include <Eigen/Core>
#include <string>
#include <math.h>
#include <truenorth/wqueue.h>
#include <truenorth/gyro_data.h>
#include <truenorth/att_consumer.h>
#include <truenorth/bias_consumer.h>
#include <truenorth/log_consumer.h>
#include <truenorth/parse_params.h>
#include <truenorth/thread.h>
#include <boost/bind.hpp>



int main(int argc, char **argv)
{

  
    // initialize node
    ros::init(argc, argv, "truenorth");

    // must initialize with "~" for param passing
    ros::NodeHandle n;


    /************************************************************************
     * Load in params
     ************************************************************************/

    estimator_params params = load_params(n);

    ros::Rate loop_rate(params.rate);
    

    /***********************************************************************
     * INITIALIZE SERIAL PORT, INITIALIZE ATT/BIAS ESTIMATION AND LOGGING THREADS
     ***********************************************************************/

    SerialPort serial(params.hz);

    AttConsumerThread* att_thread   = new AttConsumerThread(serial.att_queue,params.k.head(6),params.R0*params.R_align,params.lat,params.hz);

    BiasConsumerThread* bias_thread = new BiasConsumerThread(att_thread,serial.bias_queue,params.R_align,params.k.tail(4),params.lat);

    LogConsumerThread* log_thread   = new LogConsumerThread(bias_thread,serial.log_queue,params.log_location.c_str());
    


    /**********************************************************************
     * INITIALIZE PUBLISHER AND SUBSCRIBER
     **********************************************************************/
    // initialize publisher
    ros::Publisher chatter = n.advertise<truenorth::gyro_sensor_data>("gyro_data",1000);

    // init phins sub
    ros::Subscriber sub    = n.subscribe("phins_data",1,&BiasConsumerThread::callback, bias_thread);


   
    /***********************************************************************
     * START SERIAL PORT, START ATT/BIAS ESTIMATION AND LOGGING THREADS
     ***********************************************************************/

    bias_thread->start();
    log_thread->start();
    att_thread->start();

    // connect to serial port
    bool connected =  serial.start(params.port.c_str(),params.baud);

    if (connected==false)
    {
	ROS_ERROR("port not opened... shutting down program");
	/*bias_thread->detach();
	att_thread->detach();
	log_thread->detach();
	serial.stop();
	return 0;*/
    }
    else 
    {
	ROS_INFO("connected to port: %s",params.port.c_str());
    }


    /************************************************************      
     * MAIN LOOP
     ************************************************************/

    // initialize data_msg
    truenorth::gyro_sensor_data data_msg;

    // main loop
    while (ros::ok())
    {

      // warn if queues are growing large
      int queue_warn_size = 1000;
      if (serial.att_queue.size()>queue_warn_size)
      {
	ROS_WARN("Att queue exceeds %d - Size: :%d",queue_warn_size,serial.att_queue.size());
      }
      if (serial.bias_queue.size()>queue_warn_size)
      {
	ROS_WARN("Bias Estimation queue exceeds %d - Size: :%d",queue_warn_size,serial.bias_queue.size());
      }
      if (serial.log_queue.size()>queue_warn_size)
      {
	ROS_WARN("Logging queue exceeds %d - Size: :%d",queue_warn_size,serial.log_queue.size());
      }

      
      // fill data_msg with data packet
      pthread_mutex_lock(&mutex_bias);
      pthread_mutex_lock(&mutex_att);

      
      for (int i=0;i<3;i++)
      {
	data_msg.kvh.imu.ang.at(i) = serial.data.ang(i);
	data_msg.kvh.imu.acc.at(i) = serial.data.acc(i);
	data_msg.kvh.imu.mag.at(i) = serial.data.mag(i);

	data_msg.att.at(i) = 180*rot2rph((att_thread->R_ni)*params.R_align.transpose())(i)/M_PI;
	data_msg.bias.ang.at(i) = bias_thread->bias.w_b(i);
	data_msg.bias.acc.at(i) = bias_thread->bias.a_b(i);
	data_msg.bias.z.at(i)   = bias_thread->bias.z(i);

      }
      pthread_mutex_unlock(&mutex_att);
      pthread_mutex_unlock(&mutex_bias);


      for (int i=0;i<6;i++)
      {
	data_msg.kvh.status.at(i) = serial.data.status.at(i);
      }
	
      data_msg.kvh.temp = serial.data.temp;
      data_msg.kvh.stamp = serial.data.timestamp;
      data_msg.kvh.seq_num = serial.data.seq_num;

      // publish packet
      chatter.publish(data_msg);
  
      ros::spinOnce();
      
      loop_rate.sleep();

    }

    bias_thread->detach();
    att_thread->detach();
    log_thread->detach();
    serial.stop();
    return 0;
}
