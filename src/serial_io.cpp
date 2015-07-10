/*
 * serial_io.cpp
 * node for publishing sensor data
 *
 * created July 2015
 * Andrew Spielvogel
 * andrewspielvogel@gmail.com
 */


#include <kvh_1775/serial_io.h>
#include "ros/ros.h"
#include <boost/crc.hpp>
#include <bitset>



// set kvh 1775 data packet values
void KVHData::set_values(std::vector<float> a, std::vector<float> w, float m_t, std::vector<bool> stat, unsigned int num)
{
    int skipped = abs(seq_num-num);

    if (skipped>1&&skipped<127)
    {

	ROS_WARN("Lost %u data packets",skipped);

    }

    ang = w;
    acc = a;
    seq_num = num;
    status = stat;

    if (num>127)
    {
	std::vector<float> m;
	m.push_back(m_t);
	m.push_back(m_t);
	m.push_back(m_t);
	mag = m;
	temp = m_t;
    }
    else
    {
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

	    mag.at(0) = m_t;

	}
	else if (mod == 2)
	{

	    mag.at(1) = m_t;

	}
	else if (mod == 3)
	{

	    mag.at(2) = m_t;

	}

    }
    
    
}


union FloatSignals
{
    char c[4];
    float f;
};


// parse data packet into fields
void parse_data(KVHData &data, char *data_raw)
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
    std::vector<float> w;
    w.push_back(wx.f);
    w.push_back(wy.f);
    w.push_back(wz.f);

    // store acc data in a vec
    std::vector<float> a;
    a.push_back(ax.f);
    a.push_back(ay.f);
    a.push_back(az.f);

    // store status data in a vec
    std::bitset<8> stat(((unsigned char *) data_raw)[32]);
    std::vector<bool> status;
    status.push_back(stat[0]==1);
    status.push_back(stat[1]==1);
    status.push_back(stat[2]==1);
    status.push_back(stat[4]==1);
    status.push_back(stat[5]==1);
    status.push_back(stat[6]==1);

    // store sequence number
    unsigned int seq_num = (unsigned int) (((unsigned char *) data_raw)[33]);
 
    // set data struct with new values
    data.set_values(a, w, m_t.f, status, seq_num);


}


SerialPort::SerialPort(void)
{
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
 
    // option settings...
    port_->set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
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



