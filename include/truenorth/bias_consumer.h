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
  wqueue<GyroData>& m_queue_; /**< Queue. */
  Eigen::Matrix3d R_align_; /**< PHINS to KVH rotation. */
  Eigen::Matrix3d Rni_; /**< PHINS attitude. */
  int start_; /**< Start Bias estimator when PHINS data valid. */

 
 public:
  BiasEst bias; /**< Bias estimator. */
  Eigen::Matrix3d Rni; /**< KVH attitude (from PHINS). */
  /**
   * @brief Constructor.
   * 
   * @param queue Queue to consume from.
   * @param k Attitude estimation gains.
   * @param lat Latitude.
   * 
   */
 BiasConsumerThread(wqueue<GyroData>& queue, Eigen::Matrix3d R_align, Eigen::VectorXd k, float lat) : m_queue_(queue),bias(k,lat) {R_align_ = R_align; Rni_ <<1,0,0,0,1,0,0,0,1; Rni<<1,0,0,0,1,0,0,0,1; start_ = 0;}

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
  start_ = 1;

}
 
  void* run()
  {


    // Remove 1 item at a time and process it. Blocks if no items are 
    // available to process.
    for (int i = 0;; i++)
    {
      GyroData item = m_queue_.remove();
            
      pthread_mutex_lock(&mutex_phins);
      Rni = Rni_;
      pthread_mutex_unlock(&mutex_phins);

      if (start_)
      {
	pthread_mutex_lock(&mutex_bias);

	bias.step(Rni*R_align,item.ang,item.acc,item.mag,item.diff);
	
	pthread_mutex_unlock(&mutex_bias);

      }
      
    }
    return NULL;
  }
};

#endif

