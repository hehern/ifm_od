#include "camera_points.h"

namespace Camera
{
Camera_points::Camera_points(ros::NodeHandle nh,ros::NodeHandle pnh)
{
    std::string topic;
    pnh.param("x_offset", x_offset, 0.0);
    pnh.param("port_number", port_number, 42000);
    pnh.param("devip_str", devip_str, std::string("192.168.250.164"));
    pnh.param("frame_id", frame_id, std::string("myframe"));
    pnh.param("pcl_topic", topic, std::string("/IFM/ifm_points_left"));
    pcl_pub = nh.advertise<sensor_msgs::PointCloud2>(topic,1);
    od_result_pub = nh.advertise<driver_ifmod::objectData_t_ch8_msg>("/IFM/od_result",1);
    calibration_pub = nh.advertise<driver_ifmod::calibration_msg>("/IFM/calibration",1);
    logic_pub = nh.advertise<driver_ifmod::logic_msg>("/IFM/logic",1);
    marker_pub = nh.advertise<visualization_msgs::MarkerArray>("/IFM/marker",1);

    ros::Rate r(10);
    int channelOfInterest;
    pnh.param("channelOfInterest", channelOfInterest, 8);

    udpget.startUDPConnection(port_number, devip_str);
    while(ros::ok())
    {
        //ROS_INFO("channelOfInterest:%d",channelOfInterest);
        if(udpget.Receiver(channelOfInterest) == RESULT_OK)
        {
            pcl_pixel_publish();
            od_result_publish();
            calibration_result_publish();
            logic_publish();
            marker_publish();
        }
        //r.sleep();
    }
    udpget.stopUDPConnection();
}
void Camera_points::pcl_pixel_publish()
{
    sensor_msgs::PointCloud2 output;
    VPointCloud cloud;
    output.header.frame_id = frame_id;
    cloud.width = 16;
    cloud.height = 64;
    cloud.points.resize(cloud.width * cloud.height);
    //填充点云数据
    for(int i=0;i<NUM_SENSOR_PIXELS;i++)
    {
        //0=valid, 1=invalid
        //The lowest bit of the confidence contains the information if the pixel is valid
        //if(((udpget.g_distanceData.confidence[i] & 1) == 0)&&((udpget.g_distanceData.confidence[i] & 64) != 0))
        //{
            cloud.points[i].x = udpget.g_distanceData.X[i];
            cloud.points[i].y = udpget.g_distanceData.Y[i];
            cloud.points[i].z = udpget.g_distanceData.Z[i];
            cloud.points[i].intensity = udpget.g_distanceData.ampl_norm[i];
            cloud.points[i].ring = 0;
        //}
        
    }
    pcl::toROSMsg(cloud,output);
    output.header.frame_id = frame_id;
    pcl_pub.publish(output);
}
void Camera_points::od_result_publish()
{
    driver_ifmod::objectData_t_ch8_msg object_msg;
    for(int i=0; i<20; i++)
    {
        object_msg.objectList[i].id = udpget.g_objectData_ch8.objectList[i].id;
        object_msg.objectList[i].history = udpget.g_objectData_ch8.objectList[i].history;
        object_msg.objectList[i].measured = udpget.g_objectData_ch8.objectList[i].measured;
        object_msg.objectList[i].type = udpget.g_objectData_ch8.objectList[i].type;
        object_msg.objectList[i].age = udpget.g_objectData_ch8.objectList[i].age;
        object_msg.objectList[i].x1 = udpget.g_objectData_ch8.objectList[i].x1;
        object_msg.objectList[i].y1 = udpget.g_objectData_ch8.objectList[i].y1;
        object_msg.objectList[i].x2 = udpget.g_objectData_ch8.objectList[i].x2;
        object_msg.objectList[i].y2 = udpget.g_objectData_ch8.objectList[i].y2;
        object_msg.objectList[i].zMin = udpget.g_objectData_ch8.objectList[i].zMin;
        object_msg.objectList[i].zMax = udpget.g_objectData_ch8.objectList[i].zMax;
        object_msg.objectList[i].vX = udpget.g_objectData_ch8.objectList[i].vX;
        object_msg.objectList[i].vY = udpget.g_objectData_ch8.objectList[i].vY;
        object_msg.objectList[i].vZ = udpget.g_objectData_ch8.objectList[i].vZ;
        object_msg.objectList[i].aX = udpget.g_objectData_ch8.objectList[i].aX;
        object_msg.objectList[i].aY = udpget.g_objectData_ch8.objectList[i].aY;
        object_msg.objectList[i].aZ = udpget.g_objectData_ch8.objectList[i].aZ;
        object_msg.objectList[i].existenceProbability = udpget.g_objectData_ch8.objectList[i].existenceProbability;
        object_msg.objectList[i].vXQuality = udpget.g_objectData_ch8.objectList[i].vXQuality;
        object_msg.objectList[i].vYQuality = udpget.g_objectData_ch8.objectList[i].vYQuality;
        object_msg.objectList[i].distanceToEgo = udpget.g_objectData_ch8.objectList[i].distanceToEgo;
    }
    object_msg.crashPredictorResult.crashPredicted = udpget.g_objectData_ch8.crashPredictorResult.crashPredicted;
    object_msg.crashPredictorResult.crashPredictTime = udpget.g_objectData_ch8.crashPredictorResult.crashPredictTime;
    object_msg.crashPredictorResult.relImpactVelocity = udpget.g_objectData_ch8.crashPredictorResult.relImpactVelocity;
    object_msg.crashPredictorResult.crashObjectID = udpget.g_objectData_ch8.crashPredictorResult.crashObjectID;
    object_msg.crashPredictorResult.criticality = udpget.g_objectData_ch8.crashPredictorResult.criticality;

    od_result_pub.publish(object_msg);
}
void Camera_points::calibration_result_publish()
{
    driver_ifmod::calibration_msg cali_msg;
    cali_msg.calibValid = udpget.g_calibrationResult.commonCalibrationResult.calibValid;
    cali_msg.calibrationStableCounter = udpget.g_calibrationResult.commonCalibrationResult.calibrationStableCounter;
    cali_msg.calibResult.transX = udpget.g_calibrationResult.commonCalibrationResult.calibResult.transX;
    cali_msg.calibResult.transY = udpget.g_calibrationResult.commonCalibrationResult.calibResult.transY;
    cali_msg.calibResult.transZ = udpget.g_calibrationResult.commonCalibrationResult.calibResult.transZ;
    cali_msg.calibResult.rotX = udpget.g_calibrationResult.commonCalibrationResult.calibResult.rotX;
    cali_msg.calibResult.rotY = udpget.g_calibrationResult.commonCalibrationResult.calibResult.rotY;
    cali_msg.calibResult.rotZ = udpget.g_calibrationResult.commonCalibrationResult.calibResult.rotZ;

    calibration_pub.publish(cali_msg);
}
void Camera_points::logic_publish()
{
    driver_ifmod::logic_msg log_msg;
    for(int i=0; i<100; i++)
    {
        log_msg.digitalOutput[i] = udpget.g_logicOutput.digitalOutput[i];
    }
    for(int j=0; j<20; j++)
    {
        log_msg.analogOutput[j] = udpget.g_logicOutput.analogOutput[j];
    }
    logic_pub.publish(log_msg);
}
void Camera_points::marker_publish()
{
    visualization_msgs::MarkerArray marker_array;
    int i = 0;
    int k = 0;
    while(i<20)
    {
        //可能性大于0.5才显示
        if(udpget.g_objectData_ch8.objectList[i].existenceProbability > 0.5)
        {
            visualization_msgs::Marker line_strip, text;
            line_strip.header.frame_id = text.header.frame_id = frame_id;
            line_strip.header.stamp = text.header.stamp = ros::Time::now();
            line_strip.ns = text.ns = "driver_ifmod";
            line_strip.action = text.action = visualization_msgs::Marker::ADD;
            line_strip.id = k++;
            text.id = k++;
            line_strip.type = visualization_msgs::Marker::LINE_STRIP;
            text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            line_strip.scale.x = 0.05;
            line_strip.color.r = 1.0;
            line_strip.color.a = 1.0;
            text.scale.z = 0.4;
            text.color.b = 1;
            text.color.g = 1;
            text.color.r = 1;
            text.color.a = 1;
            //填充边界点云位置
            geometry_msgs::Point p1,p2,p3,p4;
            p1.x = udpget.g_objectData_ch8.objectList[i].x1 + x_offset;
            p1.y = udpget.g_objectData_ch8.objectList[i].y1;
            p1.z = udpget.g_objectData_ch8.objectList[i].zMax;

            p2.x = udpget.g_objectData_ch8.objectList[i].x2 + x_offset;
            p2.y = udpget.g_objectData_ch8.objectList[i].y2;
            p2.z = udpget.g_objectData_ch8.objectList[i].zMax;

            p3.x = udpget.g_objectData_ch8.objectList[i].x2 + x_offset;
            p3.y = udpget.g_objectData_ch8.objectList[i].y2;
            p3.z = udpget.g_objectData_ch8.objectList[i].zMin;

            p4.x = udpget.g_objectData_ch8.objectList[i].x1 + x_offset;
            p4.y = udpget.g_objectData_ch8.objectList[i].y1;
            p4.z = udpget.g_objectData_ch8.objectList[i].zMin;

            line_strip.points.push_back(p1);
            line_strip.points.push_back(p2);
            line_strip.points.push_back(p3);
            line_strip.points.push_back(p4);
            line_strip.points.push_back(p1);

            //填充text消息
            geometry_msgs::Pose pose;
            pose.position.x = (udpget.g_objectData_ch8.objectList[i].x1 + udpget.g_objectData_ch8.objectList[i].x2)/2;
            pose.position.y = (udpget.g_objectData_ch8.objectList[i].y1 + udpget.g_objectData_ch8.objectList[i].y2)/2;
            pose.position.z = (udpget.g_objectData_ch8.objectList[i].zMin + udpget.g_objectData_ch8.objectList[i].zMax)/2;
            text.pose = pose;
            std::ostringstream str;
            str << udpget.g_objectData_ch8.objectList[i].id;
            text.text=str.str();
            
            //pushback
            marker_array.markers.push_back(line_strip);
            marker_array.markers.push_back(text);
        }
        
        i++; 
    }
    marker_pub.publish(marker_array);
}
}//namespace Camera

