#include "scholar.h"

double linear_speed = 0;
double angular_speed = 0;	

void cmdCallback(const geometry_msgs::Twist& msg)
{
	linear_speed = msg.linear.x ;
	angular_speed = msg.angular.z;
        
}
    
int main(int argc, char** argv)
{

	ros::init(argc, argv, "scholar_bringup");	
        fflush(stdout);						
	ros::NodeHandle nh("~");  

        std::string port_name; 
        std::string ns; 
        int port_rate;
        unsigned int port_rate__;

        double linear_speed_calibrate_factor;
        double angular_speed_calibrate_factor;

        float wheel_diameter;
        float wheel_track;
        tf2_ros::TransformBroadcaster broadcast_tf; 


        nh.getParam("port_name", port_name);
        nh.getParam("port_rate", port_rate);
        nh.getParam("linear_speed_calibrate_factor", linear_speed_calibrate_factor);
        nh.getParam("angular_speed_calibrate_factor", angular_speed_calibrate_factor);
        nh.getParam("wheel_diameter", wheel_diameter);
        nh.getParam("wheel_track", wheel_track);
        nh.getParam("ns", ns);

        port_rate__ = (unsigned int)port_rate;

        scholar::Mobile_robot robot(port_name, port_rate);

	if(robot.SCHOLAR_READY == true)
        { 
           ROS_INFO("Scholar Robot initialized successful.");
           robot.scholar_linear_speed_factor = linear_speed_calibrate_factor;
           robot.scholar_angular_speed_factor = angular_speed_calibrate_factor;
           robot.scholar_wheel_diameter = wheel_diameter;
           robot.scholar_wheel_track = wheel_track;


        }
	else if(!robot.SCHOLAR_READY)ROS_ERROR("Scholar Robot initialized failed.");
    
	ros::Subscriber sub = nh.subscribe("/cmd_vel", 10, &cmdCallback);
        ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>(ns + "/odom", 10);
        ros::Publisher battery_pub = nh.advertise<sensor_msgs::BatteryState>(ns + "/battery", 10);    

	ros::Rate loop_rate(20);
	while (ros::ok()) 
	{

            try
            {

                if(robot.read_msg() == true)
                {
                    odom_pub.publish(robot.odom);
                    battery_pub.publish(robot.battery);
                    broadcast_tf.sendTransform(robot.transformStamped);
                    while(robot.READ_FLAG == true)robot.READ_FLAG = false;
                }
                
                robot.spOnce(linear_speed, angular_speed);

                while(robot.WRITE_FLAG == true){robot.WRITE_FLAG = false;};
                
	        ros::spinOnce();
	        loop_rate.sleep();
            }
            catch(const std::exception& e)
            {
		ROS_ERROR("//////////////ERROR/////////////.");
            }
	}

	return 0;
}


