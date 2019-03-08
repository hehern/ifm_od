#include "can_bridge.h"

namespace Can_bridge
{
Can_bridge::Can_bridge(ros::NodeHandle nh, ros::NodeHandle pnh)
{
    std::string topic;
    pnh.param("yawrate_topic", topic, std::string("/imu_torso/xsens/data"));
    yaw_rate_sub = nh.subscribe(topic, 1, &Can_bridge::yawrate_sent_callback,this);
    pnh.param("speed_topic", topic, std::string("/Candecode_1/SpeedMilSteer"));
    speed_sub = nh.subscribe(topic, 1, &Can_bridge::speed_sent_callback,this);
}
void Can_bridge::yawrate_sent_callback(const sensor_msgs::Imu::ConstPtr &rec)
{
    double yaw_rate = (rec->angular_velocity.z + 3.92) * 8192;

    struct can_frame yaw_rate_frame;
    yaw_rate_frame.can_id = 0x18F009FE;
    yaw_rate_frame.can_dlc = 8;
    yaw_rate_frame.data[3] = static_cast<unsigned short>(yaw_rate) & 0xff;
    yaw_rate_frame.data[4] = (static_cast<unsigned short>(yaw_rate) >>8) & 0xff;
    int sent_nbytes = write(socketcan.s,&yaw_rate_frame,sizeof(yaw_rate_frame));
    if(sent_nbytes != sizeof(yaw_rate_frame))
    {
        printf("Send Error yaw_rate_frame !\n");
    }
}
void Can_bridge::speed_sent_callback(const socket_can::SpeedMilSteer::ConstPtr &rec)
{

    double speed_ms = rec->Speed;
    double speed_kmh = speed_ms * 3.6;
    double speed = speed_kmh/256.0;
    struct can_frame speed_frame;
    speed_frame.can_id = 0x1803FFFE;
    speed_frame.can_dlc = 8;
    speed_frame.data[2] = static_cast<unsigned short>(std::abs(speed)) & 0xff;
    speed_frame.data[3] = (static_cast<unsigned short>(std::abs(speed)) >> 8) & 0xff;
    int sent_nbytes = write(socketcan.s,&speed_frame,sizeof(speed_frame));
    if(sent_nbytes != sizeof(speed_frame))
    {
        printf("Send Error speed_frame !\n");
    }
    struct can_frame direction_frame;
    direction_frame.can_id = 0xCFE6CFE;
    direction_frame.can_dlc = 8;
    if(speed >= 0)   direction_frame.data[3] &= 0x3f;
    else if (speed < 0) direction_frame.data[3] = 0x7f;
    sent_nbytes = write(socketcan.s,&direction_frame,sizeof(direction_frame));
    if(sent_nbytes != sizeof(direction_frame))
    {
        printf("Send Error direction_frame !\n");
    }

    
}

}//namespace Can_bridge
int main(int argc, char ** argv)
{
    ros::init(argc,argv,"can_bridge");
	ros::NodeHandle nh;
	ros::NodeHandle pnh("~");
    //system(down);
    //system(command);
    //system(up);
    Can_bridge::Can_bridge can_topic_bridge(nh,pnh);
	ros::spin();
    return 0;
}