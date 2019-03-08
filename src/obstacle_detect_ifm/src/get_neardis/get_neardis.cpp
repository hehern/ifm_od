#include "get_neardis.h"

namespace get_neardis
{
Get_neardis::Get_neardis(ros::NodeHandle node_handle, ros::NodeHandle private_node_handle)
{  
    private_node_handle.param("neardis_num_threshold", neardis_num_threshold_, 6);
    private_node_handle.param("neardis_dis_threshold", neardis_dis_threshold_, 0.3);
    private_node_handle.param("wheelbase", wheelbase_, 2.84);
    private_node_handle.param("front_overhang", front_overhang_, 1.1);
}

float Get_neardis::GetNearestDistance(VPointCloud::Ptr pcl_in)
{  
    VPointCloud::Ptr in(new VPointCloud(*pcl_in));
    const size_t  pcl_size = in->points.size();
    if(pcl_size > 0)
    {
        size_t cout = 0;
        float distance;
        while(cout < pcl_size)
        {
            cout++;
            //Initialize to maximum float
            float near_point_x = std::numeric_limits<double>::max();
            //find the mininum x point
            int num_cout = 0;
            for(size_t i = 0; i < pcl_size; i++)
            {
                if((std::isfinite(in->points[0].x)) && (in->points[i].x < near_point_x))
                {
                    near_point_x = in->points[i].x;
                    num_cout = i;
                }
            }
            //judge if it is a bunch of points
            int neardis_num = CoutNumOfNearestNeighborPoints(num_cout, in);
            //if it is a obstacle, return the average distance of the car
            //else set the point to nan
            if(neardis_num > neardis_num_threshold_) 
            {
                return distance = near_point_x - wheelbase_ - front_overhang_;
            }
            else{
                in->points[num_cout].x = NAN;
                in->points[num_cout].y = NAN;
                in->points[num_cout].z = NAN;
            }
        }
    }
}
int Get_neardis::CoutNumOfNearestNeighborPoints(const int num_couter, VPointCloud::Ptr &pcl_in)
{
    const size_t  pcl_size = pcl_in->points.size();
    const VPoint &p_1 = pcl_in->points[num_couter];
    int neardis_num = 0;
    for(int i = 0; i < pcl_size, i != num_couter; i++)
    {
        if(std::isfinite(pcl_in->points[i].x) && std::isfinite(pcl_in->points[i].y) && std::isfinite(pcl_in->points[i].z))
        {
            const VPoint &p_2 = pcl_in->points[i];
            double dis2 = X2(p_1.x - p_2.x) + X2(p_1.y - p_2.y) + X2(p_1.z - p_2.z);
            if (dis2 < X2(neardis_dis_threshold_))  neardis_num++;
        }       
    }
    return neardis_num;
}
}//namespace get_neardis
