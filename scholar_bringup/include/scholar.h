#ifndef MOBILE_ROBOT_H
#define MOBILE_ROBOT_H

#include <ros/ros.h>
#include <ros/time.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <std_msgs/Int32.h>
#include <sensor_msgs/BatteryState.h>

namespace scholar
{        
        const float PI = 3.1416;

	class Mobile_robot
	{
		public:
		    Mobile_robot(std::string port_name, unsigned int port_rate);
		    ~Mobile_robot();
                   
		    bool init(std::string port_name, unsigned int port_rate); 

                    bool spOnce(double linear_speed, double angular_speed);
                    bool read_msg();	
                    void handle_read( char *buf, boost::system::error_code ec, std::size_t bytes_transferred );
                    void listen_data(int max_seconds);

                public:

		    bool READ_FLAG;
                    bool WRITE_FLAG;
                    bool SCHOLAR_READY;
                    sensor_msgs::BatteryState battery;
                    nav_msgs::Odometry odom;
                    geometry_msgs::TransformStamped transformStamped;

                    float scholar_linear_speed_factor;
                    float scholar_angular_speed_factor;

                    float scholar_wheel_diameter;
                    float scholar_wheel_track;


		private:

                    bool readSpeedCommand();                     
		    void writeSpeedCommand(double linear_speed, double angular_speed);
                    
		private:

		    ros::Time current_time, last_time;

                    boost::asio::io_service iosev;
                    boost::asio::serial_port *sp;

                    boost::system::error_code ec;
                  
                    int READ_BUFFER_SIZE;
 
                    bool exception;

		    double x;
		    double y;
		    double th;

		    double vx;
		    double vy;
		    double vth;


	};
    
}

#endif /* MOBILE_ROBOT_H */
