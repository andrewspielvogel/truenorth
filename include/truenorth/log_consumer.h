/** 
 * @file
 * @date March 2017.
 * @author Andrew Spielvogel (andrewspielvogel@gmail.com).
 * @brief Consumer thread for doing IMU data logging.
 */


#ifndef LOG_CONSUMER_H
#define LOG_CONSUMER_H

#include "wqueue.h"
#include "thread.h"
#include <ros/ros.h>
#include <Eigen/Core>
#include "bias_consumer.h"

/**
 * @brief Class for consumer thread doing attitude estimation.
 */
class LogConsumerThread : public Thread
{
 private:
  FILE *fp_; /**< Log file. */
  wqueue<GyroData> &m_queue_; /**< Queue.*/
  BiasConsumerThread* bias_thread_;
  Eigen::Matrix3d Rni_;
 
 public:

  /**
   * @brief Constructor.
   * 
   * @param queue Queue to consume from.
   * @param log_location Log location.
   * 
   */
 LogConsumerThread(BiasConsumerThread * & bias_thread,wqueue<GyroData> &queue, std::string log_location) : m_queue_(queue), bias_thread_(bias_thread)
 {
   // get current time to name log file
   time_t now = time(0);
   tm *time = localtime(&now);
    
   int year = 1900 +time->tm_year;
   int month = 1 + time->tm_mon;
   int day = time->tm_mday;
   int hour = time->tm_hour;
   int minute = 1 + time->tm_min;
  
   char file_name [128];
   sprintf(file_name,"%s%d_%d_%d_%d_%d.KVH",log_location.c_str(),year,month,day,hour,minute);

   ROS_INFO("Logging IMU Data to: %s",file_name);

   // open log file
   fp_ = fopen(file_name,"w");

   Rni_<<1,0,0,0,1,0,0,0,1;
  } 

  void* run() {

    // Remove 1 item at a time and process it. Blocks if no items are 
    // available to process.
    for (int i = 0;; i++)
    {

      //log data
      

      GyroData data = m_queue_.remove();


      
      pthread_mutex_lock(&mutex_phins);
      Rni_ = bias_thread_->Rni;
      pthread_mutex_unlock(&mutex_phins);

      fprintf(fp_,"IMU_RAW, %.40f,%.40f,%.40f, %.35f,%.35f,%.35f, %.30f,%.30f,%.30f, %f, %d, %.30f,%.30f, %d, %d, %d, %d, %d, %d,%f,%f,%f,%f,%f,%f,%f,%f,%f \n",
      	      data.ang(0),data.ang(1),data.ang(2),data.acc(0),data.acc(1),data.acc(2),data.mag(0),data.mag(1),data.mag(2),data.temp,
      	      data.seq_num,data.timestamp,data.comp_timestamp,(int) data.status.at(0),(int) data.status.at(1),(int) data.status.at(2),
      	      (int) data.status.at(3),(int) data.status.at(4),(int) data.status.at(5),Rni_(0,0),Rni_(0,1),Rni_(0,2),Rni_(1,0),Rni_(1,1),Rni_(1,2),Rni_(2,0),Rni_(2,1),Rni_(2,2));
    }
    return NULL;
  }
};

#endif
