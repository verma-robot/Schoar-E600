#ifndef SCHOLAR_IMU_H
#define SCHOLAR_IMU_H

#include <ros/ros.h>
#include <ros/time.h>
#include <boost/asio.hpp>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <iostream>



namespace scholar_imu
{


       typedef union
       {
          unsigned char u[4];
          int32_t f;

       }uTof;

	class IMU_DRIVER
	{
		public:
		    IMU_DRIVER(std::string port_name, unsigned int port_rate);
		    ~IMU_DRIVER();

                    sensor_msgs::Imu imu_data;

		    bool init(std:: string port_name, int port_rate);
		    bool readOnce();

                   
                    bool read_msg();	
                    bool IMU_READY;
                    
                    
		private:
		    ros::Time current_time, last_time;

                    boost::asio::io_service iosev;
                    boost::asio::serial_port *sp;

                    boost::system::error_code ec;

                    //std::string port_name; 
                    //int port_rate; 
                    double acc_g;
    
                    bool exception;

                    int READ_BUFFER_SIZE;

		    double acc_x;
		    double acc_y;
		    double acc_z;		    
                    uTof temp_data,acc_,ang_,si_;




	};
    
}

#endif /* SCHOLAR_IMU_H */
