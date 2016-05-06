/*
 * serial_io.cpp
 * implementation of serial_io.h
 * serial port i/o for KVH 1775 IMU
 *
 * created July 2015
 * Andrew Spielvogel
 * andrewspielvogel@gmail.com
 */

#include <iostream>
#include <kvh_1775/serial_io.h>
#include "ros/ros.h"
#include <boost/crc.hpp>
#include <bitset>
#include <eigen/Eigen/Core>
#include <eigen/Eigen/Geometry>
#include <ctime>



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


GyroData::GyroData(float k1_,float k2_,float k3_, float k4_)
{
  // define inialization values
  Eigen::Vector3d zero_init(0.0,0.0,0.0);
  std::vector<bool> init_stat(6,false);

  // initialize imu data to zero
  mag = zero_init;
  ang = zero_init;
  acc = zero_init;

  // initialize other fields
  temp = 0.0;
  seq_num = 500;
  status = init_stat;
  prev_time = ros::Time::now().toSec();

  // assign gains for bias estimation
  k1 = k1_;
  k2 = k2_;
  k3 = k3_;
  k4 = k4_;

  // initialize initial bias fields
  bias_acc = zero_init;
  bias_ang = zero_init;
  bias_z = zero_init;


  // get current time to name log file
  time_t now = time(0);
  tm *time = localtime(&now);

  int year = 1900 +time->tm_year;
  int month = 1 + time->tm_mon;
  int day = time->tm_mday;
  int hour = time->tm_hour;
  int minute = 1 + time->tm_min;

  char file_name [50];
  sprintf(file_name,"/var/log/KVH/%d_%d_%d_%d_%d.KVH",year,month,day,hour,minute);
  
  // open log file
  fp_ = fopen(file_name,"w");

}

GyroData::~GyroData(void)
{

  fclose(fp_);

}

// set kvh 1775 data packet values
void GyroData::set_values(Eigen::Vector3d a, Eigen::Vector3d w, float m_t, std::vector<bool> stat, unsigned int num)
{
    int skipped = abs(seq_num-num);

    // check for lost data packets
    if (skipped>1&&skipped<127)
    {

	ROS_WARN("Lost %u data packets",skipped);

    }

    // set values
    ang = w;
    acc = a;
    seq_num = num;
    status = stat;
    prev_time = ros::Time::now().toSec();


    // choose which data was sent
    // mod == 0 -- temp
    // mod == 1 -- magx
    // mod == 2 -- magy
    // mod == 3 -- magz
    int mod = num % 4;
    
    if (mod == 0)
      {

	temp = m_t;

      }
    else if (mod == 1)
      {

	mag(0) = m_t;

      }
    else if (mod == 2)
      {

	mag(1) = m_t;

      }
    else if (mod == 3)
      {

	mag(2) = m_t;

      }   
    

    //log data
    fprintf(fp_,"RAW, %f,%f,%f, %f,%f,%f, %f,%f,%f, %f, %d \n",ang(0),ang(1),ang(2),acc(0),acc(1),acc(2),mag(0),mag(1),mag(2),temp,seq_num);

}

void GyroData::est_bias()
{
  
  if(seq_num>200)
  {

    acc_est = acc;

  }
  else
  {
    // get dt and da
    double dt = 1.0/1000.0; //ros::Time::now().toSec() - prev_time;
    Eigen::Vector3d da = acc_est - acc;

    // calculate dx
    Eigen::Vector3d da_est = -ang.cross(acc_est) + ang.cross(bias_acc) - acc.cross(bias_ang) - bias_z - k1*da;
    Eigen::Vector3d dab = k2*ang.cross(da);
    Eigen::Vector3d dwb = -k3*acc.cross(da);
    Eigen::Vector3d dzb = k4*da;
 
    // calculate next bias estimate 
    acc_est = acc_est + dt*da_est;
    bias_acc = bias_acc + dt*dab;
    bias_ang = bias_ang + dt*dwb;
    bias_z = bias_z + dt*dzb;

    }

}


union FloatSignals
{
    char c[4];
    float f;
};


// parse data packet into fields
void parse_data(GyroData &data, char *data_raw)
{

   
    FloatSignals wx, wy, wz, ax, ay, az, m_t;
    

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

    // set data struct with new values
    data.set_values(a, w, m_t.f, status, seq_num);

    //run integration, will need to add storage of previous estimate in GyroData class
    data.est_bias();
 
}


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
 
    int fd;

    fd = port_->native_handle();

    struct termios2 tio;
    ioctl(fd, TCGETS2, &tio);
    tio.c_cflag &= ~CBAUD;
    tio.c_cflag |= BOTHER;
    tio.c_ispeed = baud_rate;
    tio.c_ospeed = baud_rate;
    /* do other miscellaneous setup options with the flags here */
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

    for (unsigned int i = 0; i < bytes_transferred; ++i) 
    {

	unsigned int c = ((unsigned char *) read_buf_raw_)[i];

	// look for start sequence
	if (state_==0&&c==254)
	{
	    state_=1;
	    data_buf_raw_[0] = read_buf_raw_[i];
	}
	else if (state_==1&&c==129)
	{
	    state_=2;
	    data_buf_raw_[1] = read_buf_raw_[i];
	}
	else if (state_==2&&c==255)
	{
	    state_=3;
	    data_buf_raw_[2] = read_buf_raw_[i];
	}
	else if (state_==3&&c==87)
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
		    
		    // parse data
		    parse_data(data,data_buf_raw_);
			    
		}
		else
		{

		    ROS_WARN("corrupted package");

		}
			

	    }
	}
	else 
	{
	    ROS_WARN("lost byte: %02X",c);

	    data_cnt_=0;
	    state_=0;
	}

    }

    async_read_some_();
}



