#include <vector>
#include "imu.h"

namespace scholar_imu
{

boost::array<double, 9> vel_covariance = {
    {4.44e-5, 0, 0, 
    0, 5.87e-5, 0, 
    0, 0, 0.000150723
    }};

boost::array<double, 9> ang_covariance = {
    {2.54e-7, 0, 0, 
    0, 5.97e-7, 0, 
    0, 0, 2.54e-7
    }};

boost::array<double, 9> pose_covariance = {
    {3.64e-6, 0, 0, 
    0, 4.64e-6, 0, 
    0, 0, 2.2e-7
    }};

IMU_DRIVER::IMU_DRIVER(std::string port_name, unsigned int port_rate):sp(NULL)
{ 
    sp = new boost::asio::serial_port(iosev);
    if(sp)init(port_name, port_rate);
}

IMU_DRIVER::~IMU_DRIVER()
{
   if(sp)delete sp;

   if (sp) {
     sp -> cancel();
     sp -> close();

  }

  iosev.stop();
  iosev.reset();

  delete sp;

}

bool IMU_DRIVER::init(std::string port_name, int port_rate)
{

         try {

              sp -> open(port_name, ec); 
              if(ec)
              {       
                  ROS_ERROR( "Can't open serial port") ;
                  throw ec;
               }
              sp -> set_option(boost::asio::serial_port::baud_rate(port_rate));
	      sp -> set_option(boost::asio::serial_port::flow_control(boost::asio::serial_port::flow_control::none));
	      sp -> set_option(boost::asio::serial_port::parity(boost::asio::serial_port::parity::none));
	      sp -> set_option(boost::asio::serial_port::stop_bits(boost::asio::serial_port::stop_bits::one));
	      sp -> set_option(boost::asio::serial_port::character_size(8));

              ros::Time::init();
	      current_time = ros::Time::now();
	      last_time = ros::Time::now();
              IMU_READY = true;
              return true;

         }
         catch(...) {

              IMU_READY = false;
              ROS_ERROR( "Can't open serial port") ;
              return false;
         }       

}

bool IMU_DRIVER::read_msg()
{

    uint8_t length, buffer_data[60], serial_data[53];


    tcflush(1, TCIOFLUSH);
    try{

         length=boost::asio::read(*sp, boost::asio::buffer(buffer_data), ec);

         if(length >= 53)
         {

             for(int i = 0; i < 53; i ++)
             {

                if(buffer_data[i] == 89 && buffer_data[i + 1] == 83 && buffer_data[i + 4] == 46)
                 {

                  for(int j = 0; j < 53; j ++)
                    {
                      serial_data[j] = buffer_data[i + j];
                    }
                                  
                 }
             } 

             if(serial_data[34] == 16 && serial_data[33] == 65)// siyuanshu
             {

                    
                    temp_data.u[0] = serial_data[35];
                    temp_data.u[1] = serial_data[36];
                    temp_data.u[2] = serial_data[37];
                    temp_data.u[3] = serial_data[38];
                    imu_data.orientation.w = temp_data.f * 0.000001;
                    temp_data.u[0] = serial_data[39];
                    temp_data.u[1] = serial_data[40];
                    temp_data.u[2] = serial_data[41];
                    temp_data.u[3] = serial_data[42];
                    imu_data.orientation.x = temp_data.f * 0.000001;
                    temp_data.u[0] = serial_data[43];
                    temp_data.u[1] = serial_data[44];
                    temp_data.u[2] = serial_data[45];
                    temp_data.u[3] = serial_data[46];
                    imu_data.orientation.y = temp_data.f * 0.000001;
                    temp_data.u[0] = serial_data[47];
                    temp_data.u[1] = serial_data[48];
                    temp_data.u[2] = serial_data[49];
                    temp_data.u[3] = serial_data[50];
                    imu_data.orientation.z = temp_data.f * 0.000001;

             }


             if(serial_data[6] == 12 && serial_data[5] == 16)//linear acceleration
             {

                    temp_data.u[0] = serial_data[7];
                    temp_data.u[1] = serial_data[8];
                    temp_data.u[2] = serial_data[9];
                    temp_data.u[3] = serial_data[10];
                    acc_x = temp_data.f * 0.000001;
                    temp_data.u[0] = serial_data[11];
                    temp_data.u[1] = serial_data[12];
                    temp_data.u[2] = serial_data[13];
                    temp_data.u[3] = serial_data[14];
                    acc_y = temp_data.f * 0.000001;
                    temp_data.u[0] = serial_data[15];
                    temp_data.u[1] = serial_data[16];
                    temp_data.u[2] = serial_data[17];
                    temp_data.u[3] = serial_data[18];
                    acc_z = temp_data.f * 0.000001;

                    imu_data.linear_acceleration.x = acc_x;

                    imu_data.linear_acceleration.y = acc_y;

                    imu_data.linear_acceleration.z = acc_z;

             }

             if(serial_data[20]==12 && serial_data[19]==32)//angular vel
             {


                    temp_data.u[0] = serial_data[21];
                    temp_data.u[1] = serial_data[22];
                    temp_data.u[2] = serial_data[23];
                    temp_data.u[3] = serial_data[24];
                    imu_data.angular_velocity.x = temp_data.f * 0.000001 * 3.1416 / 180;
                    temp_data.u[0] = serial_data[25];
                    temp_data.u[1] = serial_data[26];
                    temp_data.u[2] = serial_data[27];
                    temp_data.u[3] = serial_data[28];
                    imu_data.angular_velocity.y = temp_data.f * 0.000001 * 3.1416 / 180;
                    temp_data.u[0] = serial_data[29];
                    temp_data.u[1] = serial_data[30];
                    temp_data.u[2] = serial_data[31];
                    temp_data.u[3] = serial_data[32];
                    imu_data.angular_velocity.z = temp_data.f * 0.000001 * 3.1416 / 180;

       
                     
             }


             imu_data.header.stamp = ros::Time::now();
             imu_data.header.frame_id = "imu_link";
             imu_data.linear_acceleration_covariance = vel_covariance;
             imu_data.angular_velocity_covariance = ang_covariance;
             imu_data.orientation_covariance = pose_covariance;  
  
             return true;

      }   
       
     else 
        {

            throw exception;
         } 
    }
    catch(bool  exception)
    {
      ROS_ERROR("SCHOLAR IMU ERROR DATA");

      return false;

    }     

}

}
