#ifndef CONSUMER_H
#define CONSUMER_H

#include "wqueue.h"
#include "gyro_data.h"
#include "thread.h"
#include <ros/ros.h>


class ConsumerThread : public Thread
{
    wqueue<GyroData*>& m_queue;
 
  public:
    ConsumerThread(wqueue<GyroData*>& queue) : m_queue(queue) {}
 
    void* run() {
        // Remove 1 item at a time and process it. Blocks if no items are 
        // available to process.
        for (int i = 0;; i++)
	{
	  GyroData* item = m_queue.remove();
	  if (item->seq_num == 1)
	  {
	    //ROS_ERROR("%d",item->seq_num);
	  }

	  //delete item;
        }
        return NULL;
    }
};

#endif
