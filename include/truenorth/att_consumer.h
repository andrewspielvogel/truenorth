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
#include <ros/ros.h>
#include <Eigen/Core>
#include "bias_est.h"
#include "parse_params.h"

/**
 * @brief Class for consumer thread doing attitude estimation.
 */
class AttConsumerThread : public Thread
{

 private:
  wqueue<GyroData>& m_queue_; /**< Queue.*/
  estimator_params params_;

 
 public:
  Eigen::Matrix3d R_ni; /**< Attitude Estimate. */
  AttEst att; /**< Attitude estimation class.*/


  /**
   * @brief Constructor.
   * 
   * @param queue Queue to consume from.
   * @param k Attitude estimation gains.
   * @param R_align Initial attitude alignment rotation.
   * @param lat Latitude.
   * @param hz Sampling rate.
   * 
   */
 AttConsumerThread(wqueue<GyroData>& queue, estimator_params params) : m_queue_(queue), att(params.k.head(6),params.R0*params.R_align,params.lat)
  {
  R_ni = params.R0*params.R_align;
  params_ = params;
  } 


  /**
   * Consumer thread run function.
   */
  void* run() {

    // Remove 1 item at a time and process it. Blocks if no items are 
    // available to process.
    for (int i = 0;; i++)
    {
      GyroData item = m_queue_.remove();
      

      att.step(item.ang,item.acc,item.diff);

      pthread_mutex_lock(&mutex_att);
      R_ni = att.att.R_ni;
      pthread_mutex_unlock(&mutex_att);
      
    }
    return NULL;
  }
};

#endif
