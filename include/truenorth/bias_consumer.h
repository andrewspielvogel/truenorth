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
#include <math.h>
#include <phins/phins_msg.h>
#include <pthread.h>

static pthread_mutex_t mutex_phins;

/**
 * @brief Class for consumer thread doing bias estimation.
 */
class BiasConsumerThread : public Thread
{
 private:
  wqueue<GyroData*>& m_queue_; /**< Queue. */
  BiasEst bias_;
  Eigen::Matrix3d R_align_;
  Eigen::Matrix3d Rni_;

 
 public:
  BiasEst bias;
  Eigen::Matrix3d Rni;
  /**
   * @brief Constructor.
   * 
   * @param queue Queue to consume from.
   * @param k Attitude estimation gains.
   * @param lat Latitude.
   * 
   */
 BiasConsumerThread(wqueue<GyroData*>& queue, Eigen::VectorXd k, float lat) : m_queue_(queue), bias_(k,lat), bias(k,lat) {R_align_<<1,0,0,0,-1,0,0,0,-1; Rni_ <<1,0,0,0,1,0,0,0,1;}

  void callback(const phins::phins_msg::ConstPtr &msg)
{

  double r = (msg->att.at(0))*M_PI/180;
  double p = (msg->att.at(1))*M_PI/180;
  double h = (msg->att.at(2))*M_PI/180;
  pthread_mutex_lock(&mutex_phins);
  Rni_ << cos(h)*cos(p),cos(h)*sin(p)*sin(r) - sin(h)*cos(r),cos(h)*sin(p)*cos(r) + sin(h)*sin(r),
    sin(h)*cos(p),sin(h)*sin(p)*sin(r) + cos(h)*cos(r),sin(h)*sin(p)*cos(r) - cos(h)*sin(r),
    -sin(p),cos(p)*sin(r),cos(p)*cos(r);
  pthread_mutex_unlock(&mutex_phins);

}
 
  void* run()
  {

    /*
     * TODO!!!!!!!!!!!
     * NEED TO ADD FUNCTIONALITY TO PASS IN Rni(t). RIGHT NOW Rni(t) IS STATIC.
     */
    

    // Remove 1 item at a time and process it. Blocks if no items are 
    // available to process.
    for (int i = 0;; i++)
    {
      GyroData* item = m_queue_.remove();
      
      pthread_mutex_lock(&mutex_bias);
      bias = bias_;
      pthread_mutex_unlock(&mutex_bias);
      
      pthread_mutex_lock(&mutex_phins);
      Rni = Rni_;
      pthread_mutex_unlock(&mutex_phins);

      bias_.step(Rni*R_align_,item->ang,item->acc,item->diff);

      //delete item;
    }
    return NULL;
  }
};

#endif

