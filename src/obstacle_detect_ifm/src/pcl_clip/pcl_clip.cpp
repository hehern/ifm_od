#include "pcl_clip.h"

namespace pcl_clip
{
Pcl_clip::Pcl_clip(ros::NodeHandle node_handle, ros::NodeHandle private_node_handle)
{  
    private_node_handle.param("roi_x_min", roi_x_min_, 0.0);
    private_node_handle.param("roi_x_max", roi_x_max_, 10.0);
    private_node_handle.param("roi_y_min", roi_y_min_, -5.0);
    private_node_handle.param("roi_y_max", roi_y_max_, 5.0);
    private_node_handle.param("roi_z_min", roi_z_min_, -1.0);
    private_node_handle.param("roi_z_max", roi_z_max_, 10.0);
}
VPointCloud::Ptr Pcl_clip::GetPcl(const VPointCloud::ConstPtr in)
{
    if(in->points.size() > 0)
    {
        VPointCloud::Ptr out(new VPointCloud);
        for (const VPoint &p : in->points)
        {
            if (IsIn(p.x, roi_x_min_, roi_x_max_) && IsIn(p.y, roi_y_min_, roi_y_max_) && IsIn(p.z, roi_z_min_, roi_z_max_))
            out->push_back(p);
        }
        return out;
    }
    return nullptr; 
}

}//namespace remove_ground
