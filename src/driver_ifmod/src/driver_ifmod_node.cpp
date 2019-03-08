#include "driver_ifmod_node.h"

int main(int argc, char *argv[])
{
    ros::init(argc,argv,"driver_ifmod_node");
	ros::NodeHandle node_handle;
	ros::NodeHandle privete_node_handle("~");
    Camera::Camera_points camera_points(node_handle,privete_node_handle);
	//ros::spin();
    return 0;
}
