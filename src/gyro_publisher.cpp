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
#include <truenorth/log_consumer.h>
#include <truenorth/parse_params.h>
#include <truenorth/thread.h>
#include <boost/bind.hpp>


/**
 *
 * @brief Class for launching attitude estimation rosnode.
 * 
 */
class AttNode
{

private:
  ros::Publisher chatter_; /**< Node publisher. */
  ros::Subscriber sub_; /**< Node subscriber to PHINS topic. */
  
public:
  SerialPort* serial; /**< Serial port object for reading IMU data. */
  AttConsumerThread* att_thread; /**< Attitude estimation thread. */
  LogConsumerThread* log_thread; /**< Logging thread. */
  estimator_params params; /**< Node parameters. */


  /**
   *
   * @brief Constructor.
   * 
   * @param n ROS NodeHandle
   * 
   */
  AttNode(ros::NodeHandle n){

    params       = load_params(n);
    serial       = new SerialPort(params.hz);
    att_thread   = new AttConsumerThread(serial->att_queue,params.k.head(6),params.R0*params.R_align,params.lat,params.hz);


    log_thread   = new LogConsumerThread(serial->log_queue,params.log_location.c_str());
    chatter_     = n.advertise<truenorth::gyro_sensor_data>("gyro_data",1000);

    // init phins sub
    sub_ = n.subscribe("phins_data",1,&LogConsumerThread::phins_callback, log_thread);

  }

  /**
   *
   * @brief Publishing callback function.
   * 
   */
  void timerCallback(const ros::TimerEvent&)
  {


    // initialize data_msg
    truenorth::gyro_sensor_data data_msg;


    // warn if queues are growing large
    int queue_warn_size = 1000;
    if (serial->att_queue.size()>queue_warn_size)
    {
      ROS_WARN("Att queue exceeds %d - Size: :%d",queue_warn_size,serial->att_queue.size());
    }
    if (serial->log_queue.size()>queue_warn_size)
    {
      ROS_WARN("Logging queue exceeds %d - Size: :%d",queue_warn_size,serial->log_queue.size());
    }

      
    // fill data_msg with data packet
    pthread_mutex_lock(&mutex_att);

      
    for (int i=0;i<3;i++)
      {
	data_msg.kvh.imu.ang.at(i) = serial->data.ang(i);
	data_msg.kvh.imu.acc.at(i) = serial->data.acc(i);
	data_msg.kvh.imu.mag.at(i) = serial->data.mag(i);

	data_msg.att.at(i) = 180.0*rot2rph((att_thread->R_ni)*params.R_align.transpose())(i)/M_PI;
	data_msg.bias.ang.at(i) = att_thread->att.w_b(i);
	data_msg.bias.acc.at(i) = att_thread->att.a_b(i);
	data_msg.bias.z.at(i)   = att_thread->att.w_E_north(i);

      }
    pthread_mutex_unlock(&mutex_att);


    for (int i=0;i<6;i++)
      {
	data_msg.kvh.status.at(i) = serial->data.status.at(i);
      }
	
    data_msg.kvh.temp = serial->data.temp;
    data_msg.kvh.stamp = serial->data.timestamp;
    data_msg.kvh.seq_num = serial->data.seq_num;

    // publish packet
    chatter_.publish(data_msg);
  
  }
 
};

int main(int argc, char **argv)
{

  
    // initialize node
    ros::init(argc, argv, "truenorth");

    // must initialize with "~" for param passing
    ros::NodeHandle n;


    /**********************************************************************
     * Initialize Att Estimator
     **********************************************************************/
    AttNode att_est(n);
    
  
    /***********************************************************************
     * START ATT AND LOGGING THREADS
     ***********************************************************************/

    att_est.log_thread->start();
    att_est.att_thread->start();


    /***********************************************************************
     * Start Serial Thread
     ***********************************************************************/
    bool connected =  att_est.serial->start(att_est.params.port.c_str(),att_est.params.baud);

    if (connected==false)
    {
	ROS_ERROR("port not opened... shutting down program");
    }
    else 
    {
	ROS_INFO("connected to port: %s",att_est.params.port.c_str());
    }


    /************************************************************      
     * Main Publishing Callback
     ************************************************************/
    ros::Timer timer = n.createTimer(ros::Duration(1.0/att_est.params.rate),&AttNode::timerCallback,&att_est);

    
    ros::spin();
    
    

    att_est.att_thread->detach();
    att_est.log_thread->detach();
    att_est.serial->stop();
    return 0;
}
