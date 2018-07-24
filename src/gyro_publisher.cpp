
/**
 * @file
 * @date July 2015
 * @author Andrew Spielvogel (andrewspielvogel@gmail.com)
 * @brief Node for publishing sensor data.
 */


#include <ros/ros.h>
#include <truenorth/serial_io.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <dscl_msgs/KvhImu.h>
#include <dscl_msgs/ImuBias.h>
#include <helper_funcs/helper_funcs.h>
#include <Eigen/Core>
#include <string>
#include <math.h>
#include <truenorth/wqueue.h>
#include <helper_funcs/gyro_data.h>
#include <truenorth/att_consumer.h>
#include <truenorth/log_consumer.h>
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
  ros::Publisher chatter_; /**< IMU data publisher. */
  ros::Publisher chatter_bias_; /**< IMU bias publisher. */
  ros::Publisher chatter_att_; /**< IMU attitude publisher. */

  
public:
  SerialPort* serial; /**< Serial port object for reading IMU data. */
  AttConsumerThread* att_thread; /**< Attitude estimation thread. */
  LogConsumerThread* log_thread; /**< Logging thread. */
  config_params params; /**< Node parameters. */


  /**
   *
   * @brief Constructor.
   * 
   * @param n ROS NodeHandle
   * 
   */
  AttNode(ros::NodeHandle n){

    ROS_INFO("Loading estimator params.");

    std::string k_g;
    std::string k_north;
    std::string k_acc;
    std::string k_acc_bias;
    std::string k_ang_bias;
    std::string k_E_n;
    std::string ang_bias;
    std::string acc_bias;
    
    std::string lat;
    
    std::string r0;
    std::string r_align;

    Eigen::Vector3d rpy;
    Eigen::Vector3d vec;
    
    n.param<std::string>("k_g",k_g, "[1,1,1]");
    n.param<std::string>("k_north",k_north, "[1,1,1]");
    n.param<std::string>("k_acc",k_acc,"[1,1,1]");
    n.param<std::string>("k_acc_bias",k_acc_bias,"[1,1,1]");
    n.param<std::string>("k_ang_bias",k_ang_bias,"[1,1,1]");
    n.param<std::string>("k_E_n",k_E_n,"[1,1,1]");
    n.param<std::string>("r0",r0,"[0,0,0]");
    n.param<std::string>("r_align",r_align,"[0,0,0]");
    n.param<std::string>("ang_bias",ang_bias,"[0.000009,0,-0.000002]");
    n.param<std::string>("acc_bias",acc_bias,"[-0.095,0,0]");

    n.param<std::string>("lat",lat,"39.32");
    n.param<int>("hz",params.hz,1000);
    n.param<int>("rate",params.rate,10);
    n.param<int>("baud",params.baud,4147200);
    n.param<std::string>("port",params.port,"/dev/ttyUSB1");
    n.param<std::string>("log_loc",params.log_location,"/log/kvh");

    
    params.K_g          = stringToDiag(k_g);
    params.K_north      = stringToDiag(k_north);
    params.K_acc        = stringToDiag(k_acc);
    params.K_acc_bias   = stringToDiag(k_acc_bias);
    params.K_ang_bias   = stringToDiag(k_ang_bias);
    params.K_E_n        = stringToDiag(k_E_n);

    sscanf(ang_bias.c_str(),"[%lf,%lf,%lf]",&vec(0),&vec(1),&vec(2));
    params.ang_bias     = vec;

    sscanf(acc_bias.c_str(),"[%lf,%lf,%lf]",&vec(0),&vec(1),&vec(2));
    params.acc_bias     = vec;
    
    sscanf(r0.c_str(),"%lf,%lf,%lf",&rpy(0),&rpy(1),&rpy(2));
    params.R0           = rpy2rot(rpy);

    sscanf(r_align.c_str(),"%lf,%lf,%lf",&rpy(0),&rpy(1),&rpy(2));
    params.R_align      = rpy2rot(rpy);
    
    params.lat          = std::stod(lat);

    serial       = new SerialPort(params.hz);
    att_thread   = new AttConsumerThread(serial->att_queue,params);

    log_thread   = new LogConsumerThread(serial->log_queue);
    chatter_      = n.advertise<dscl_msgs::KvhImu>("imu",1);
    chatter_bias_ = n.advertise<dscl_msgs::ImuBias>("bias",1);
    chatter_att_  = n.advertise<geometry_msgs::Vector3Stamped>("rpy",1);

  }

  /**
   *
   * @brief Publishing callback function.
   * 
   */
  void timerCallback(const ros::TimerEvent&)
  {


    // initialize data_msg
    dscl_msgs::KvhImu imu_data;
    dscl_msgs::ImuBias imu_bias;
    geometry_msgs::Vector3Stamped att;


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

      
    imu_data.imu.ang.x = serial->data.ang(0);
    imu_data.imu.ang.y = serial->data.ang(1);
    imu_data.imu.ang.z = serial->data.ang(2);
    imu_data.imu.acc.x = serial->data.acc(0);
    imu_data.imu.acc.y = serial->data.acc(1);
    imu_data.imu.acc.z = serial->data.acc(2);
    imu_data.imu.mag.x = serial->data.mag(0);
    imu_data.imu.mag.y = serial->data.mag(1);
    imu_data.imu.mag.z = serial->data.mag(2);

    Eigen::Vector3d rph = 180.0*rot2rph((att_thread->R_ni)*params.R_align.transpose())/M_PI;
    att.vector.x = rph(0);
    att.vector.y = rph(1);
    att.vector.z = rph(2);
    
    imu_bias.ang.x = att_thread->att.bias.w_b(0);
    imu_bias.ang.y = att_thread->att.bias.w_b(1);
    imu_bias.ang.z = att_thread->att.bias.w_b(2);
    imu_bias.acc.x = att_thread->att.bias.a_b(0);
    imu_bias.acc.y = att_thread->att.bias.a_b(1);
    imu_bias.acc.z = att_thread->att.bias.a_b(2);
    imu_bias.mag.x = -1;
    imu_bias.mag.y = -1;
    imu_bias.mag.z = -1;
      
    pthread_mutex_unlock(&mutex_att);


    for (int i=0;i<6;i++)
      {
        imu_data.status.at(i) = serial->data.status.at(i);
      }
	
    imu_data.temp = serial->data.temp;
    imu_data.stamp = serial->data.timestamp;
    imu_data.seq_num = serial->data.seq_num;

    imu_data.header.stamp = ros::Time::now();
    att.header.stamp      = ros::Time::now();
    imu_bias.header.stamp = ros::Time::now();

    // publish packet
    chatter_.publish(imu_data);
    chatter_bias_.publish(imu_bias);
    chatter_att_.publish(att);
  
  }
 
};

int main(int argc, char **argv)
{

  
    // initialize node
    ros::init(argc, argv, "truenorth");

    // must initialize with "~" for param passing
    ros::NodeHandle n("~");


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
