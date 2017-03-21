/** 
 * @file
 * @date March 2017.
 * @author Andrew Spielvogel (andrewspielvogel@gmail.com).
 * @brief Consumer thread for doing bias estimation. (Based on the work 
 * done by Spielvogel and Whitcomb.
 */

#ifndef BIAS_CONSUMER_H
#define BIAS_CONSUMER_H

#include "wqueue.h"
#include "bias_est.h"
#include "thread.h"
#include <ros/ros.h>
#include <Eigen/Core>

/**
 * @brief Class for consumer thread doing bias estimation.
 */
class BiasConsumerThread : public Thread
{
  wqueue<GyroData*>& m_queue; /**< Queue. */
 
 public:
  /**
   * @brief Constructor.
   * 
   * @param queue Queue to consume from.
   * @param k Attitude estimation gains.
   * @param lat Latitude.
   * @param hz Sampling rate.
   * 
   */
 BiasConsumerThread(wqueue<GyroData*>& queue, Eigen::VectorXd k, float lat, float hz) : m_queue(queue), bias(k,lat) {}

  BiasEst bias;
 
  void* run()
  {

    /*
     * TODO!!!!!!!!!!!
     * NEED TO ADD FUNCTIONALITY TO PASS IN Rni(t). RIGHT NOW Rni(t) IS STATIC.
     */
    Eigen::Matrix3d Rni;
    Rni << 1,0,0,0,-1,0,0,0,-1;

    // Remove 1 item at a time and process it. Blocks if no items are 
    // available to process.
    for (int i = 0;; i++)
    {
      GyroData* item = m_queue.remove();
      bias.step(Rni,item->ang,item->acc,item->diff);

      //delete item;
    }
    return NULL;
  }
};

#endif
