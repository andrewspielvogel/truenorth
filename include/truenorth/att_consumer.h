/** 
 * @file
 * @date March 2017.
 * @author Andrew Spielvogel (andrewspielvogel@gmail.com).
 * @brief Consumer thread for doing attitude estimation. (Based on the work 
 * done by Spielvogel and Whitcomb.
 */


#ifndef ATT_CONSUMER_H
#define ATT_CONSUMER_H

#include "wqueue.h"
#include "att_est.h"
#include "thread.h"
#include "bias_consumer.h"
#include <ros/ros.h>
#include <Eigen/Core>
#include "bias_est.h"

/**
 * @brief Class for consumer thread doing attitude estimation.
 */
class AttConsumerThread : public Thread
{

 private:
  wqueue<GyroData>& m_queue_; /**< Queue.*/
  BiasConsumerThread* bias_thread_; /**< Bias consumer thread.*/
  AttEst att_; /**< Attitude estimation class.*/

 
 public:
  Eigen::Matrix3d R_ni; /**< Attitude Estimate. */

  /**
   * @brief Constructor.
   * 
   * @param bias_thread Bias estimation thread.
   * @param queue Queue to consume from.
   * @param k Attitude estimation gains.
   * @param R_align Initial attitude alignment rotation.
   * @param lat Latitude.
   * @param hz Sampling rate.
   * 
   */
 AttConsumerThread(BiasConsumerThread* & bias_thread, wqueue<GyroData>& queue, Eigen::VectorXd k, Eigen::Matrix3d R_align, float lat, float hz) : m_queue_(queue), att_(k,R_align,lat,hz), bias_thread_(bias_thread) {} 


  /**
   * Consumer thread run function.
   */
  void* run() {

    // Remove 1 item at a time and process it. Blocks if no items are 
    // available to process.
    for (int i = 0;; i++)
    {
      GyroData item = m_queue_.remove();
      
      //pthread_mutex_lock(&mutex_bias);

      //bias_.w_b <<5.32/1000000.0,2.3/100000.0,5.1/10000000.0;
      //bias_.a_b << 0.0026,-0.0043,0.0018;
      //att_.step(item.ang-bias_thread->bias.w_b,item.acc-bias_thread->bias.a_b,item.diff);
      att_.step(item.ang,item.acc,item.diff);
      //pthread_mutex_unlock(&mutex_bias);

      pthread_mutex_lock(&mutex_att);
      R_ni = att_.R_ni;
      pthread_mutex_unlock(&mutex_att);
      
    }
    return NULL;
  }
};

#endif
