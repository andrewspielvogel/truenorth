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
#include <kvh_1775/gyro_data.h>

typedef boost::shared_ptr<boost::asio::serial_port> serial_port_ptr;

#define SERIAL_PORT_READ_BUF_SIZE 1
#define DATA_BUF_SIZE 38


// struct for doing custom baud rate 
typedef unsigned char	cc_t;
typedef unsigned int	speed_t;
typedef unsigned int	tcflag_t;

#define NCCS 19
#define BOTHER 0010000
struct termios2 {
	tcflag_t c_iflag;		/* input mode flags */
	tcflag_t c_oflag;		/* output mode flags */
	tcflag_t c_cflag;		/* control mode flags */
	tcflag_t c_lflag;		/* local mode flags */
	cc_t c_line;			/* line discipline */
	cc_t c_cc[NCCS];		/* control characters */
	speed_t c_ispeed;		/* input speed */
	speed_t c_ospeed;		/* output speed */
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
    Eigen::Vector3d mag;
    float temp;
    SerialPort(float k1_,float k2_,float k3_,float k4_,float k5_, Eigen::Matrix3d align_): data(k1_,k2_,k3_,k4_,k5_,align_){};
    virtual ~SerialPort(void);

    virtual bool start(const char *com_port_name, int baud_rate=9600);
    virtual void stop();

protected:
    virtual void async_read_some_();
    virtual void on_receive_(const boost::system::error_code& ec, size_t bytes_transferred);
    void parse_data(GyroData &data, char *data_raw);




};


#endif
