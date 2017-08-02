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
#include "att_consumer.h"
#include <pthread.h>

static pthread_mutex_t mutex_phins; 

/**
 * @brief Class for consumer thread doing bias estimation.
 */
class BiasConsumerThread : public Thread
{
 private:
  AttConsumerThread* att_thread_; /**< Attitude consumer thread.*/
  wqueue<GyroData>& m_queue_; /**< Queue. */
  Eigen::Matrix3d R_align_; /**< PHINS to KVH rotation. */
  int start_; /**< Start Bias estimator when PHINS data valid. */

 
 public:
  BiasEst bias; /**< Bias estimator. */
  Eigen::Matrix3d Rni; /**< KVH attitude. */
  Eigen::Matrix3d Rni_; /**< PHINS attitude. */

  /**
   * @brief Constructor.
   * 
   * @param queue Queue to consume from.
   * @param k Attitude estimation gains.
   * @param lat Latitude.
   * 
   */
 BiasConsumerThread(AttConsumerThread* & att_thread,wqueue<GyroData>& queue, Eigen::Matrix3d R_align, Eigen::VectorXd k, float lat) : m_queue_(queue),bias(k,lat), att_thread_(att_thread)
  {
    R_align_ = R_align;
    Rni_ <<1,0,0,0,1,0,0,0,1;
    Rni<<1,0,0,0,1,0,0,0,1;
    start_ = 0;
  }

  /**
   * @brief Callback function for subscribing to PHINS topic.
   *
   * @param msg PHINS message.
   */
  void callback(const phins::phins_msg::ConstPtr &msg)
  {
  
    double r = (msg->att.at(0))*M_PI/180;
    double p = (msg->att.at(1))*M_PI/180;
    double h = (msg->att.at(2))*M_PI/180;
    Eigen::Vector3d rpy_phins(r,p,h);

    pthread_mutex_lock(&mutex_phins);

    Rni_ = rpy2rot(rpy_phins);

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

      /* pthread_mutex_lock(&mutex_phins); */

      /* Rni = Rni_; */

      /* pthread_mutex_unlock(&mutex_phins); */
   
      /* if (start_) */
      /* { */
      /* 	pthread_mutex_lock(&mutex_bias); */

      /* 	bias.step(Rni*R_align_,item.ang,item.acc,item.mag,item.diff); */
	
      /* 	pthread_mutex_unlock(&mutex_bias); */

      /* } */


      

      pthread_mutex_lock(&mutex_att);
      Rni = att_thread_->R_ni;
      pthread_mutex_unlock(&mutex_att);
      
      pthread_mutex_lock(&mutex_bias);

      bias.step(Rni,item.ang,item.acc,item.mag,item.diff);
 
      pthread_mutex_unlock(&mutex_bias);
      
    }
    return NULL;
  }
};

#endif

