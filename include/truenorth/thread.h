/**
 * @file
 * @date March 2017
 * @author Andrew Spielvogel (andrewspielvogel@gmail.com)
 * @brief Thread Class
 */

#ifndef THREAD_H
#define THREAD_H

#include <pthread.h>
#include <semaphore.h>

static sem_t semaphore; /**< Semaphore for blocking access to thread elements. */

/**
 * @brief Thread class bassed on pthread.
 */
class Thread
{
 public:
  Thread(); 
  virtual ~Thread(); 


  int start(); 
  int join();
  int detach();
  pthread_t self(); 
 
  virtual void* run() = 0;
 
  private:
    pthread_t  m_tid;
    int        m_running;
    int        m_detached;
};

#endif
