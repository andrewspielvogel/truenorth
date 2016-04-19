/*
 * serial_io.h
 * serial port i/o for KVH 1775 IMU
 *
 * created July 2015
 * Andrew Spielvogel
 * andrewspielvogel@gmail.com
 */


#ifndef SERIAL_IO_H
#define SERIAL_IO_H

#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/system/error_code.hpp>
#include <boost/system/system_error.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/asio/read.hpp>
#include <eigen/Eigen/Core>

typedef boost::shared_ptr<boost::asio::serial_port> serial_port_ptr;

#define SERIAL_PORT_READ_BUF_SIZE 1
#define DATA_BUF_SIZE 38


//class for storing a KVH 1775 data packet
class GyroData
{
public:
    Eigen::Vector3d ang;
    Eigen::Vector3d acc;
    Eigen::Vector3d mag;
    std::vector<bool> status;
    float temp;
    unsigned int seq_num;

    double prev_time;
    Eigen::Vector3d bias_acc;
    Eigen::Vector3d bias_ang;
    Eigen::Vector3d bias_z;
    Eigen::Vector3d acc_est;

    void est_bias();
    float k1,k2,k3,k4;

    GyroData(float k1_,float k2_,float k3_,float k4_);
    virtual ~GyroData(void);
    void set_values (Eigen::Vector3d, Eigen::Vector3d, float, std::vector<bool>, unsigned int);

};



// class for connecting to a serial port
// and for getting data from a serial port
class SerialPort
{
protected:
    boost::asio::io_service io_service_;
    serial_port_ptr port_;
    boost::mutex mutex_;

    char read_buf_raw_[SERIAL_PORT_READ_BUF_SIZE];
    char data_buf_raw_[DATA_BUF_SIZE];


    int state_;
    int data_cnt_;
  
private:
    SerialPort(const SerialPort &p);
    SerialPort &operator=(const SerialPort &p); 
 
public:
    GyroData data;
    SerialPort(float k1_,float k2_,float k3_,float k4_): data(k1_,k2_,k3_,k4_){};
    virtual ~SerialPort(void);

    virtual bool start(const char *com_port_name, int baud_rate=9600);
    virtual void stop();

protected:
    virtual void async_read_some_();
    virtual void on_receive_(const boost::system::error_code& ec, size_t bytes_transferred);




};


#endif
