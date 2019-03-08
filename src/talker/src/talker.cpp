#include <ros/ros.h>
#include <can_msgs/Frame.h>
#include <sensor_msgs/Imu.h>



int main( int argc, char** argv )
{
  ros::init(argc, argv, "talker");                         
  ros::NodeHandle n;
  ros::Publisher publisher = n.advertise<sensor_msgs::Imu>("/imu_torso/xsens/data", 1);   
 

  ros::Rate r(30);

  while (ros::ok())
  {

    /*can_msgs::Frame::Ptr result(new can_msgs::Frame());
    result->header.stamp = ros::Time::now();
    result->id = 0x182;
    result->is_rtr = false;
    result->is_extended = false;
    result->dlc = 8;
    result->data[0] = 0x00;
    result->data[1] = 0x00;
    result->data[2] = 0xf4;
    result->data[3] = 0x01;
    result->data[4] = 0x00;
    result->data[5] = 0x00;
    result->data[6] = 0x00;
    result->data[7] = 0x00;
    can_publisher.publish(result);*/
    sensor_msgs::Imu::Ptr result(new sensor_msgs::Imu);
    result->angular_velocity.z = 3.62;
    publisher.publish(result);
    r.sleep();
  }
}
