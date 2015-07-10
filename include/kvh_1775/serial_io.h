#ifndef SERIAL_IO_H
#define SERIAL_IO_H

#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/system/error_code.hpp>
#include <boost/system/system_error.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/asio/read.hpp>

typedef boost::shared_ptr<boost::asio::serial_port> serial_port_ptr;

#define SERIAL_PORT_READ_BUF_SIZE 1
#define DATA_BUF_SIZE 38


class KVHData
{
public:
    std::vector<float> ang;
    std::vector<float> acc;
    std::vector<float> mag;
    std::vector<bool> status;
    float temp;
    unsigned int seq_num;

    void set_values (std::vector<float>, std::vector<float>, float, std::vector<bool>, unsigned int);

};




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
    char data_[DATA_BUF_SIZE];
  
private:
    SerialPort(const SerialPort &p);
    SerialPort &operator=(const SerialPort &p); 
 
public:
    SerialPort(void);
    virtual ~SerialPort(void);

    virtual bool start(const char *com_port_name, int baud_rate=9600);
    virtual void stop();

    KVHData data;

    

protected:
    virtual void async_read_some_();
    virtual void on_receive_(const boost::system::error_code& ec, size_t bytes_transferred);




};


#endif
