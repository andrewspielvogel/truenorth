#ifndef CONSUMER_H
#define CONSUMER_H

#include "wqueue.h"
#include "att_est.h"
#include "thread.h"
#include <ros/ros.h>
#include <Eigen/Core>


class ConsumerThread : public Thread
{
    wqueue<GyroData*>& m_queue;
 
  public:
 ConsumerThread(wqueue<GyroData*>& queue, Eigen::VectorXd k, Eigen::Matrix3d R_align, float lat, float hz) : m_queue(queue), att(k,R_align,lat,hz) {}

    AttEst att;
 
    void* run() {
      
        // Remove 1 item at a time and process it. Blocks if no items are 
        // available to process.
        for (int i = 0;; i++)
	{
	  GyroData* item = m_queue.remove();
	  att.step(item->ang-,item->acc,item->timestamp,item->diff);

	  //delete item;
	 }
	 return NULL;
    }
};

#endif
