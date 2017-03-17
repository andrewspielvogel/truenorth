#ifndef CONSUMER_H
#define CONSUMER_H

#include "wqueue.h"
#include "att_est.h"
#include "thread.h"
#include "bias_consumer.h"
#include <ros/ros.h>
#include <Eigen/Core>


class ConsumerThread : public Thread
{
    wqueue<GyroData*>& m_queue;
    BiasConsumerThread* bias;
 
  public:
 ConsumerThread(BiasConsumerThread* & bias_thread, wqueue<GyroData*>& queue, Eigen::VectorXd k, Eigen::Matrix3d R_align, float lat, float hz) : m_queue(queue), att(k,R_align,lat,hz), bias(bias_thread) {}

    AttEst att;
 
    void* run() {

        // Remove 1 item at a time and process it. Blocks if no items are 
        // available to process.
        for (int i = 0;; i++)
	{
	  GyroData* item = m_queue.remove();
	  att.step(item->ang-bias->bias.w_b,item->acc-bias->bias.a_b,item->timestamp,item->diff);

	  //delete item;
	 }
	 return NULL;
    }
};

#endif
