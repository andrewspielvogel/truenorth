/**
 * @file
 * @author Andrew Spielvogel (andrewspielvogel@gmail.com)
 * @date July 2015
 * @brief Serial port i/o for KVH 1775 IMU.
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
#include <Eigen/Core>
#include <truenorth/gyro_data.h>
#include <truenorth/wqueue.h>
#include <truenorth/thread.h>
#include <string>



typedef unsigned char	cc_t;
typedef unsigned int	speed_t;
typedef unsigned int	tcflag_t;

#define NCCS 19
#define BOTHER 0010000

/** 
 * @brief Struct for doing custom baud rate. 
 *
 * This is a hack, copied from asm/termios.h because
 * I can't also include that header since it redefines structs from
 * the normal termios header file that is used by boost
 */
struct termios2 {
	tcflag_t c_iflag;		/**< Input mode flags. */
	tcflag_t c_oflag;		/**< Output mode flags. */
	tcflag_t c_cflag;		/**< Control mode flags. */
	tcflag_t c_lflag;		/**< Local mode flags. */
	cc_t c_line;			/**< Line discipline. */
	cc_t c_cc[NCCS];		/**< Control characters. */
	speed_t c_ispeed;		/**< Input speed. */
	speed_t c_ospeed;		/**< Output speed. */
};




typedef boost::shared_ptr<boost::asio::serial_port> serial_port_ptr; /**< Serial port pointer. */

#define SERIAL_PORT_READ_BUF_SIZE 32 /**< Size of read in buffer. */


/*
 * DATA PACKET INFO
 */

// Start Sequence
#define START_SEQ_1 0xFE /**< Start sequence, first byte */
#define START_SEQ_2 0x81 /**< Start sequence, second byte */
#define START_SEQ_3 0xFF /**< Start sequence, third byte */
#define START_SEQ_4 0x57 /**< Start sequence, fourth byte - message type C */
#define DATA_BUF_SIZE 38 /**< Size of data packet (in bytes). */



/** 
 * @brief Class for connecting to a serial port
 * and for getting data from a serial port.
 */
class SerialPort
{
protected:
    boost::asio::io_service io_service_; /**< Boost io service. */
    serial_port_ptr port_; /**< Boost serial port class. */
    boost::mutex mutex_;

    char read_buf_raw_[SERIAL_PORT_READ_BUF_SIZE]; /**< Read in buffer. */
    char data_buf_raw_[DATA_BUF_SIZE]; /**< Data packet buffer. */


    int state_; /**< State of data read in. */
    int data_cnt_; /**< Track number of bytes read in. */
    
private:
    SerialPort(const SerialPort &p);
    SerialPort &operator=(const SerialPort &p);

 
public:
    GyroData data; /**< Class for storing IMU data. */
    wqueue<GyroData> att_queue; /**< Queue for attitude estimation. */
    wqueue<GyroData> bias_queue; /**< Queue for bias estimation. */
    wqueue<GyroData> log_queue; /**< Queue for logging IMU data. */

    /**
     * Constructor.
     * @param k Estimation gains. 
     * @parblock The elements in order are: Local level estimation gain, heading estimation gain,
     * linear accleration estimation gain, linear acceleration bias estimation gain, angular 
     * velocity bias estimation gain, and z bias constant estimation gain.
     * @endparblock
     * @param align Initial estimation of attitude rotation.
     * @param log_location_ Location of IMU data log file.
     */
    SerialPort(float hz);

    virtual ~SerialPort(void); /**< Destructor */

    /**
     * Start serial port communication and attitude estimation.
     *
     * Open serial port and start attitude estimation.
     * @param com_port_name Serial port to open.
     * @param baud_rate Baud rate of serial port.
     */
    virtual bool start(const char *com_port_name, int baud_rate=9600);

    /**
     * Close serial port.
     *
     * Close serial port and stop attitude estimation.
     */
    virtual void stop();

protected:
    virtual void async_read_some_(); /**< Asynchronously read in data */

    /**
     * Function called when data is received.
     *
     * Function looks for data packet start sequence, reads in data packets, and does crc checking.
     * @param err_code Boost system error code.
     * @param bytes_transferred Number of bytes received in asynchronous read.
     */
    virtual void on_receive_(const boost::system::error_code& ec, size_t bytes_transferred);

    /**
     * Parses IMU data packet.
     *
     * Parses IMU data packet into GyroData struct.
     * @param data_raw IMU data packet to be parsed.
     */
    void parse_data_(char *data_raw);



};


#endif
