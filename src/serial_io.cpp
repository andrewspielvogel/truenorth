/**
 * @file
 * @date July 2015
 * @author Andrew Spielvogel (andrewspielvogel@gmail.com)
 * 
 * @brief Implementation of serial_io.h.
 *
 * Serial port i/o for KVH 1775 IMU.
 *
 */

#include <truenorth/serial_io.h>
#include <truenorth/gyro_data.h>
#include "ros/ros.h"
#include <boost/crc.hpp>
#include <boost/thread.hpp>
#include <bitset>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <stdint.h>


/*
 *
 * SEE HEADER FILE FOR DOCUMENTATION
 *
 */

/**
 * @brief Union for storing gyro data
 */
union FloatSignals
{
  char c[4];
  float f;
  uint32_t uint32;
  uint16_t uint16;
};

#if MESSAGE_TYPE == 1

// parse data packet into fields
void SerialPort::parse_data_( char *data_raw)
{

   
  FloatSignals wx, wy, wz, ax, ay, az, timestamp, temp; //ang, acc components and timestamp
    

  // get ang, acc, and mag/temp data
  for (int i=0;i<4;i++)
    {

      wx.c[3-i] = ((unsigned char *) data_raw)[4+i];
      wy.c[3-i] = ((unsigned char *) data_raw)[8+i];
      wz.c[3-i] = ((unsigned char *) data_raw)[12+i];
    
      ax.c[3-i] = ((unsigned char *) data_raw)[16+i];
      ay.c[3-i] = ((unsigned char *) data_raw)[20+i];
      az.c[3-i] = ((unsigned char *) data_raw)[24+i];

      timestamp.c[3-i]= ((unsigned char *) data_raw)[28+i];

    }

  for (int i=0;i<2;i++)
    {

      temp.c[1-i] = ((unsigned char *) data_raw)[34+i];

    }

  // store ang data in a vec
  Eigen::Vector3d w(wx.f,wy.f,wz.f);

  // store acc data in a vec
  Eigen::Vector3d a(ax.f,ay.f,az.f);

  // store status data in a vec
  std::bitset<8> stat(((unsigned char *) data_raw)[32]);
  std::vector<bool> status;

  for(int i=0;i<7;i++)
    {
      if (i!=3){
	status.push_back(stat[i]==1);
      }
    }

  // store sequence number
  unsigned int seq_num = (unsigned int) (((unsigned char *) data_raw)[33]);


  // define time and diff
  data.diff = ((float)timestamp.uint32) - data.timestamp;
  data.timestamp = ((float)timestamp.uint32);

  if (data.seq_num > 200){

    data.t_start = data.timestamp;

  }

  // store data
  data.ang = w;
  data.acc = a;
  int skipped = abs(data.seq_num-seq_num);
  data.seq_num = seq_num;
  data.status = status;

  data.temp = temp.uint16;


  // check for lost data packets
  if (skipped>1&&skipped<127)
    {

      ROS_WARN("Lost %u data packets",skipped);

    }

  // log data
  data.log()
  //boost::thread log_thread(&GyroData::log,&data);
  //boost::thread bias_thread(&GyroData::est_bias,&data);
  //boost::thread att_thread(&GyroData::est_att,&data);
 
}


#else

// parse data packet into fields
void SerialPort::parse_data_( char *data_raw)
{

   
  FloatSignals wx, wy, wz, ax, ay, az, m_t; //ang, acc components and m_t is mag/temp depending on seq_num
    

  // get ang, acc, and mag/temp data
  for (int i=0;i<4;i++)
    {

      wx.c[3-i] = ((unsigned char *) data_raw)[4+i];
      wy.c[3-i] = ((unsigned char *) data_raw)[8+i];
      wz.c[3-i] = ((unsigned char *) data_raw)[12+i];
    
      ax.c[3-i] = ((unsigned char *) data_raw)[16+i];
      ay.c[3-i] = ((unsigned char *) data_raw)[20+i];
      az.c[3-i] = ((unsigned char *) data_raw)[24+i];

      m_t.c[3-i]= ((unsigned char *) data_raw)[28+i];

    }

  // store ang data in a vec
  Eigen::Vector3d w(wx.f,wy.f,wz.f);

  // store acc data in a vec
  Eigen::Vector3d a(ax.f,ay.f,az.f);

  // store status data in a vec
  std::bitset<8> stat(((unsigned char *) data_raw)[32]);
  std::vector<bool> status;

  for(int i=0;i<7;i++)
    {
      if (i!=3){
	status.push_back(stat[i]==1);
      }
    }

  // store sequence number
  unsigned int seq_num = (unsigned int) (((unsigned char *) data_raw)[33]);


  // choose which data was sent and store
  // mod == 0 -- temp
  // mod == 1 -- magx
  // mod == 2 -- magy
  // mod == 3 -- magz
  int mod = seq_num % 4;
    
  if (mod == 0)
    {

      data.temp = m_t.f;

    }
  else if (mod == 1)
    {

      data.mag(0) = m_t.f;

    }
  else if (mod == 2)
    {

      data.mag(1) = m_t.f;

    }
  else if (mod == 3)
    {

      data.mag(2) = m_t.f;

    }   

  // define time and diff
  int seq_diff = seq_num - data.seq_num;
  
  if (data.seq_num > 128)
  {

    seq_diff = 0;

  }
  else if (seq_diff < 0)
  {

    seq_diff += 128;

  }

  // save timestamp
  data.diff = ((double)seq_diff)/((double)data.hz);
  //data.diff = ((double)1)/((double)data.hz);

  data.timestamp += data.diff;

  // store data
  data.ang = w;
  data.acc = a;
  data.seq_num = seq_num;
  data.status = status;


  // check for lost data packets
  if (seq_diff>1)
    {

      ROS_WARN("Lost %u data packets",seq_diff);

    }

  // log data
  data.log();
  //boost::thread bias_thread(&GyroData::est_bias,&data);
  //boost::thread att_thread(&GyroData::est_att,&data);
 
}

#endif

// destructor
SerialPort::~SerialPort(void)
{
  stop();
}


// start serial connection
bool SerialPort::start(const char *com_port_name, int baud_rate)
{
  boost::system::error_code ec;
	
  state_ = 0;
  data_cnt_ = 0;

  // check if already opened
  if (port_)
    {
      ROS_ERROR("error : port is already opened...");
      return false;
    }
 
  // open port
  port_ = serial_port_ptr(new boost::asio::serial_port(io_service_));
  port_->open(com_port_name, ec);
  if (ec) 
    {
      ROS_ERROR( "error : port_->open() failed...com_port_name= %s, e = %s",
		 com_port_name , ec.message().c_str() ); 
      return false;
    }
 
  // get fd of serial port native handle
  int fd;
  fd = port_->native_handle();

  // assign custom baud rate
  struct termios2 tio;
  ioctl(fd, TCGETS2, &tio);
  tio.c_cflag &= ~CBAUD;
  tio.c_cflag |= BOTHER;
  tio.c_ispeed = baud_rate;
  tio.c_ospeed = baud_rate;
  ioctl(fd, TCSETS2, &tio);

  // option settings...
  //port_->set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
  port_->set_option(boost::asio::serial_port_base::character_size(8));
  port_->set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
  port_->set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
  port_->set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
 
  boost::thread t(boost::bind(&boost::asio::io_service::run, &io_service_));
 
  async_read_some_();
 
  return true;
}
 

// close port
void SerialPort::stop()
{
  boost::mutex::scoped_lock look(mutex_);
 
  if (port_) 
    {
      port_->cancel();
      port_->close();
      port_.reset();
    }
  io_service_.stop();
  io_service_.reset();
}


// read in bytes
void SerialPort::async_read_some_()
{
  if (port_.get() == NULL || !port_->is_open()) return;
 
  port_->async_read_some(
			 boost::asio::buffer(read_buf_raw_, SERIAL_PORT_READ_BUF_SIZE),
			 boost::bind(
				     &SerialPort::on_receive_,
				     this, boost::asio::placeholders::error, 
				     boost::asio::placeholders::bytes_transferred));
}

// function called upon byte read in
void SerialPort::on_receive_(const boost::system::error_code& ec, size_t bytes_transferred)
{
  boost::mutex::scoped_lock look(mutex_);
 
  if (port_.get() == NULL || !port_->is_open()) return;
  if (ec) 
  {
    async_read_some_();
    return;
  }

  // receive data packet
  for (unsigned int i = 0; i < bytes_transferred; ++i) 
  {

    unsigned int c = ((unsigned char *) read_buf_raw_)[i];

    // look for start sequence
    if (state_==0&&c==START_SEQ_1)
    {
      state_=1;
      data_buf_raw_[0] = read_buf_raw_[i];
    }
    else if (state_==1&&c==START_SEQ_2)
    {
      state_=2;
      data_buf_raw_[1] = read_buf_raw_[i];
    }
    else if (state_==2&&c==START_SEQ_3)
    {
      state_=3;
      data_buf_raw_[2] = read_buf_raw_[i];
    }
    else if (state_==3&&c==START_SEQ_4)
    {
      state_=4;
      data_buf_raw_[3] = read_buf_raw_[i];
    }
    else if (state_==4&&data_cnt_<DATA_BUF_SIZE)
    {
      // start sequence found
      
      // add data
      data_buf_raw_[data_cnt_+4] = read_buf_raw_[i];
      data_cnt_ +=1;
      
      // check for when correct number of data bytes read in
      if (data_cnt_>DATA_BUF_SIZE-5)
      {

	// do crc checksum on data
	boost::crc_optimal<32, 0x04C11DB7, 0xFFFFFFFF,0,false,false> checksum_agent;
	checksum_agent.process_bytes(data_buf_raw_,DATA_BUF_SIZE-4);
	unsigned int crc_calc_sum = checksum_agent.checksum();
		
	data_cnt_=0;
	state_=0;
	
	// get sent sent crc checksum
	std::bitset<8> crc_1(((unsigned char *) data_buf_raw_)[DATA_BUF_SIZE-4]);
	std::bitset<8> crc_2(((unsigned char *) data_buf_raw_)[DATA_BUF_SIZE-3]);
	std::bitset<8> crc_3(((unsigned char *) data_buf_raw_)[DATA_BUF_SIZE-2]);
	std::bitset<8> crc_4(((unsigned char *) data_buf_raw_)[DATA_BUF_SIZE-1]);
	std::bitset<32> crc_sent(crc_1.to_string() + crc_2.to_string() + crc_3.to_string() + crc_4.to_string());
	unsigned int crc_sent_sum = (unsigned int) crc_sent.to_ulong();

	// check that calculated and sent checksums are the same     
	if (crc_sent_sum == crc_calc_sum) 
	{	
		    
	  // define time and diff
	  //double time = ros::Time::now().toSec();
	  //data.diff = time - data.timestamp;
	  //data.timestamp = time;

	  // parse data
	  parse_data_(data_buf_raw_);
			    
	}
	else
	{

	  ROS_WARN("corrupted package");

	}
			

      }
    }
    else 
    {
      //ROS_WARN("lost byte: %02X",c);

      data_cnt_=0;
      state_=0;
    }

  }

  async_read_some_();
}
