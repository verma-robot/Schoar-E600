#include <vector>
#include "scholar.h"

namespace scholar
{

  boost::array<double, 36> odom_pose_covariance = {
    {1e-9, 0, 0, 0, 0, 0, 
    0, 1e-3, 1e-9, 0, 0, 0, 
    0, 0, 1e6, 0, 0, 0,
    0, 0, 0, 1e6, 0, 0, 
    0, 0, 0, 0, 1e6, 0, 
    0, 0, 0, 0, 0, 1e-9}};
  boost::array<double, 36> odom_twist_covariance = {
    {1e-9, 0, 0, 0, 0, 0, 
    0, 1e-3, 1e-9, 0, 0, 0, 
    0, 0, 1e6, 0, 0, 0, 
    0, 0, 0, 1e6, 0, 0, 
    0, 0, 0, 0, 1e6, 0, 
    0, 0, 0, 0, 0, 1e-9}};

Mobile_robot::Mobile_robot(std::string port_name, unsigned int port_rate):sp(NULL)
{  

    sp = new boost::asio::serial_port(iosev);
    if(sp)init(port_name, port_rate);
    while(WRITE_FLAG == true)WRITE_FLAG = false;
    while(READ_FLAG == true)READ_FLAG = false;

}

Mobile_robot::~Mobile_robot()
{


   if (sp) {
     sp -> cancel();
     sp -> close();
     delete sp;
  }

  iosev.stop();
  iosev.reset();
  while(WRITE_FLAG == true)WRITE_FLAG = false;
  while(READ_FLAG == true)READ_FLAG = false;


}

bool Mobile_robot::init(std::string port_name, unsigned int port_rate)
{

         try {

              sp -> open(port_name, ec); 
              //std::cout << ec << std::endl;
              if(ec)
              {       
                  ROS_ERROR( "Can't open serial port") ;
                  throw ec;
               }

              sp -> set_option(boost::asio::serial_port::baud_rate(115200));
	      sp -> set_option(boost::asio::serial_port::flow_control(boost::asio::serial_port::flow_control::none));
	      sp -> set_option(boost::asio::serial_port::parity(boost::asio::serial_port::parity::none));
	      sp -> set_option(boost::asio::serial_port::stop_bits(boost::asio::serial_port::stop_bits::one));
	      sp -> set_option(boost::asio::serial_port::character_size(8));

              ros::Time::init();
	      current_time = ros::Time::now();
	      last_time = ros::Time::now();
                

              x = 0.00;
              y = 0.00;
              th = 0.00;

              vx = 0.00;
              vy = 0.00;
              vth = 0.00;
              while(READ_FLAG == true)READ_FLAG = false;
              while(WRITE_FLAG == true)WRITE_FLAG = false;
              while(SCHOLAR_READY == false)SCHOLAR_READY = true;               

              return true;

         }
         catch(...) {

              while(READ_FLAG == true)READ_FLAG = false;
              while(WRITE_FLAG == true)WRITE_FLAG = false;
              while(SCHOLAR_READY == true)SCHOLAR_READY = false;
              //ROS_ERROR( "Serial port inited error!") ;
              return false;
         }

}


bool Mobile_robot::readSpeedCommand()
{

   uint8_t data_read[6] = {0xFF, 0x03, 0x01, 0x06,0x04, 0x6D};   

   boost::asio::write(*sp, boost::asio::buffer(&data_read[0], 6), ec);

}


void Mobile_robot::writeSpeedCommand(double linear_speed, double angular_speed)
{
	   

     uint8_t data[13] = {0xFF, 0x03, 0x01, 0x0A, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x6D};
     uint8_t check[8];

     
     float left_wheel_rate = 0.00;
     float right_wheel_rate = 0.00;

     right_wheel_rate = (60 * linear_speed + 30 * scholar_wheel_track * angular_speed * scholar_angular_speed_factor) * 10 / (PI * scholar_wheel_diameter * scholar_linear_speed_factor); // 2021.08.02
     left_wheel_rate = (60 * linear_speed - 30 * scholar_wheel_track * angular_speed * scholar_angular_speed_factor) * 10 / (PI * scholar_wheel_diameter * scholar_linear_speed_factor); //2021.08.02




     if(left_wheel_rate >= 0)
     {
           data[5] = 0x00;
     }
     else if(left_wheel_rate < 0)
     { 
           data[5] = 0x01;
     }

     left_wheel_rate = abs(left_wheel_rate);
     data[6] = ((uint16_t)left_wheel_rate) >> 8 & 0xFF;
     data[7] = ((uint16_t)left_wheel_rate) & 0xFF;


     if(right_wheel_rate >= 0)
     {
           data[8] = 0x00;
     }
     else if(right_wheel_rate < 0)
     { 
           data[8] = 0x01;
     }
     right_wheel_rate = abs(right_wheel_rate);
     data[9] = ((uint16_t)right_wheel_rate) >> 8 & 0xFF;
     data[10] = ((uint16_t)right_wheel_rate) & 0xFF;


     boost::asio::write(*sp, boost::asio::buffer(&data[0], 12), ec);

}


void Mobile_robot::handle_read( char buffer_data[], boost::system::error_code ec, std::size_t bytes_transferred )
{
    double read_left_wheel_speed = 0;
    double read_right_wheel_speed = 0;
    READ_BUFFER_SIZE = bytes_transferred;
    //std::cout << READ_BUFFER_SIZE << std::endl;
    if(READ_BUFFER_SIZE == 17)
    {   

             if ( buffer_data[1] != 0x03 || buffer_data[2] != 0x01 || buffer_data[3] != 0x0C || (buffer_data[16] != 0x6D) )
              {
                  while(WRITE_FLAG == true)WRITE_FLAG = false;
                  while(READ_FLAG == true)READ_FLAG = false;
	          ROS_ERROR("Received data error!");
              }
             else if(buffer_data[4] == 0x04 )//read
              {

                  if(buffer_data[5] == 0x00)
                  {
                       read_left_wheel_speed = -1 * (float)((buffer_data[6] << 8) + buffer_data[7]) ;
                       //read_left_wheel_speed=read_left_wheel_speed;
                  }
                  else if(buffer_data[5] == 0x01)
                  {
                       read_left_wheel_speed = 1 * (float)((buffer_data[6] << 8) + buffer_data[7]) ;
                       //read_left_wheel_speed=read_left_wheel_speed;
                  }
                  if(buffer_data[8] == 0x00)
                  {
                       read_right_wheel_speed = -1 * (float)((buffer_data[9] << 8) + buffer_data[10]) ;
                      // read_right_wheel_speed=read_right_wheel_speed;
                  }
                  else if(buffer_data[8] == 0x01)
                  {
                       read_right_wheel_speed = (float)((buffer_data[9] << 8) + buffer_data[10]) ;
                       //read_right_wheel_speed=read_right_wheel_speed;
                  }

                  battery.design_capacity = 20;
                  battery.charge = buffer_data[11] / 100.00 * battery.design_capacity;
                  battery.voltage = (buffer_data[12] << 8) + buffer_data[13];

                  battery.voltage /= 100.00;
                  battery.current = (buffer_data[14] << 8) + buffer_data[15];
                  battery.percentage = battery.charge  /  battery.design_capacity;

                  current_time = ros::Time::now();

                  vx = PI * scholar_wheel_diameter * (read_right_wheel_speed + read_left_wheel_speed) / 1200.00;

                  vx *= scholar_linear_speed_factor;//2021.08.02

                  vth = -1 * PI * scholar_wheel_diameter *scholar_linear_speed_factor * (read_right_wheel_speed - read_left_wheel_speed) / (600 * scholar_angular_speed_factor * scholar_wheel_track);//2021.08.02

//std::cout << "right_wheel: " << read_right_wheel_speed << "left_wheel: " << read_left_wheel_speed << std::endl;

//std::cout << "vx :" << vx << "vth: "<< vth << std::endl;
                  double dt = (current_time - last_time).toSec();

                  double delta_x = vx * cos(th)  * dt - vy * sin(th) * dt;
                  double delta_y = vx * sin(th) * dt + vy * cos(th) * dt;
                  double delta_th = vth * dt;

                  x += delta_x;
                  y += delta_y;
                  th += delta_th;

       
                  last_time = current_time; 
                  while(READ_FLAG == false)READ_FLAG = true;
//std::cout << READ_FLAG << std::endl;

              }
              else if(buffer_data[4] == 0x03 && buffer_data[16] == 0x6D)//write sucess
              {   

                  while(WRITE_FLAG == false)WRITE_FLAG = true;

              }              
              else if(buffer_data[4] == 0x86)//some motor discneect
              {
                  ROS_INFO("some motor disconect"); 
                  if(buffer_data[5] == 0x01)ROS_ERROR("THe left wheels DISCONEECT");
                  if(buffer_data[5] == 0x02)ROS_ERROR("The right wheels DISCONNECT");
                  if(buffer_data[5] == 0x03)ROS_ERROR("All wheels DISCONNECT");                 
                  while(WRITE_FLAG == true)WRITE_FLAG = false;
                  while(READ_FLAG == true)READ_FLAG = false;
              }
              else
              {
                  //ROS_ERROR("WRONG COMMAND ...................!");
                  while(WRITE_FLAG == true)WRITE_FLAG = false;
                  while(READ_FLAG == true)READ_FLAG = false;
                  //throw exception;
              }
    }


}

void Mobile_robot::listen_data(int max_seconds)
{

       boost::asio::deadline_timer timer( iosev); 

       char buf[17];
      
      
       iosev.reset();
       boost::asio::async_read(*sp, boost::asio::buffer(buf, sizeof(buf)), boost::bind(&Mobile_robot::handle_read, this, buf, _1,  _2)) ;      
    
       timer.expires_from_now(boost::posix_time::millisec(max_seconds)) ;      
       timer.async_wait(boost::bind(&boost::asio::serial_port::cancel, boost::ref(*sp)));


       try{

                 iosev.run();

       }
       catch(boost::system::system_error& ecc) {
                 std::cerr << ecc.what() << std::endl;

       }

}

bool Mobile_robot::read_msg()
{

          while(READ_FLAG == true)READ_FLAG = false;

          readSpeedCommand(); 
          listen_data(20);

           current_time = ros::Time::now();

           transformStamped.header.stamp = current_time;
           transformStamped.header.frame_id = "odom";
           transformStamped.child_frame_id = "base_link";
           transformStamped.transform.translation.x = x;
           transformStamped.transform.translation.y = y;
           transformStamped.transform.translation.z = 0.0;

           tf2::Quaternion q;
           q.setRPY(0, 0, th);
           transformStamped.transform.rotation.x = q.x();
           transformStamped.transform.rotation.y = q.y();
           transformStamped.transform.rotation.z = q.z();
           transformStamped.transform.rotation.w = q.w();

           odom.header.frame_id ="odom";
           odom.child_frame_id = "base_link";
           odom.header.stamp = current_time;
           odom.pose.pose.position.x = x;
           odom.pose.pose.position.y = y;
           odom.pose.pose.position.z = 0;
           odom.pose.pose.orientation.x = q.getX();
           odom.pose.pose.orientation.y = q.getY();
           odom.pose.pose.orientation.z = q.getZ();
           odom.pose.pose.orientation.w = q.getW();
           odom.twist.twist.linear.x = vx;
           odom.twist.twist.linear.y = 0;
           odom.twist.twist.angular.z = vth;

           odom.pose.covariance = odom_pose_covariance;

           odom.twist.covariance = odom_twist_covariance;


           battery.header.frame_id = "base_link";
           battery.header.stamp = current_time;

           battery.power_supply_status = 2;
           battery.power_supply_health = 0;
           battery.power_supply_technology = 3;
           battery.present = true;
    //std::cout << "publish" <<std::endl;
           return true;

}


bool Mobile_robot::spOnce(double linear_speed, double angular_speed)
{

           while(WRITE_FLAG == true)WRITE_FLAG = false;
           writeSpeedCommand(linear_speed, angular_speed);
           listen_data(50);
         
           return true;
   };



}
