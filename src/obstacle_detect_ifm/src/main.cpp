#include "top.h"

int main(int argc, char ** argv)
{
    ros::init(argc,argv,"obstacle_detect_ifm");
	ros::NodeHandle nh;
	ros::NodeHandle pnh("~");
    top::Top tp(nh,pnh);
	ros::spin();
    return 0;
}