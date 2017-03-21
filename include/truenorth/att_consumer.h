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
#include <semaphore.h>

/**
 * @brief Class for consumer thread doing attitude estimation.
 */
class AttConsumerThread : public Thread
{
  wqueue<GyroData*>& m_queue; /**< Queue.*/
  BiasConsumerThread* bias; /**< Bias consumer thread.*/
 
 public:
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
 AttConsumerThread(BiasConsumerThread* & bias_thread, wqueue<GyroData*>& queue, Eigen::VectorXd k, Eigen::Matrix3d R_align, float lat, float hz) : m_queue(queue), att(k,R_align,lat,hz), bias(bias_thread) {} 

  AttEst att; /**< Attitude estimation class*/
 
  void* run() {

    // Remove 1 item at a time and process it. Blocks if no items are 
    // available to process.
    for (int i = 0;; i++)
    {
      GyroData* item = m_queue.remove();
      sem_wait(&semaphore);
      att.step(item->ang-bias->bias.w_b,item->acc-bias->bias.a_b,item->diff);
      sem_post(&semaphore);
      
      //delete item;
    }
    return NULL;
  }
};

#endif
