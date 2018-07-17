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
#include <helper_funcs/log.h>

static pthread_mutex_t mutex_phins; 


/**
 * @brief Class for consumer thread doing attitude estimation.
 */
class LogConsumerThread : public Thread
{
 private:

  wqueue<GyroData> &m_queue_; /**< Queue.*/


 
 public:

  /**
   * @brief Constructor.
   * 
   * @param queue Queue to consume from.
   * @param log_location Log location.
   * 
   */
 LogConsumerThread(wqueue<GyroData> &queue) : m_queue_(queue)
 {

  }


  void* run() {

    // Remove 1 item at a time and process it. Blocks if no items are 
    // available to process.
    for (int i = 0;; i++)
    {

      //log data
      

      GyroData data = m_queue_.remove();


      char buffer[512];

      sprintf(buffer,"%f %f %.40f,%.40f,%.40f, %.35f,%.35f,%.35f, %.30f,%.30f,%.30f, %f, %d, %.30f,%.30f, %d, %d, %d, %d, %d, %d",rov_get_time(), ros::Time::now().toSec(),
      	      data.ang(0),data.ang(1),data.ang(2),data.acc(0),data.acc(1),data.acc(2),data.mag(0),data.mag(1),data.mag(2),data.temp,
      	      data.seq_num,data.timestamp,data.comp_timestamp,(int) data.status.at(0),(int) data.status.at(1),(int) data.status.at(2),
      	      (int) data.status.at(3),(int) data.status.at(4),(int) data.status.at(5));

      log_this_now_dsl_format(LOG_FID_KVH_FORMAT,(char *) LOG_FID_KVH_SUFFIX,buffer);

    }
    return NULL;
  }
};

#endif
