#ifndef CLIMBING_ROBOT_MAPPING
#define CLIMBING_ROBOT_MAPPING

#include <iostream>
#include <queue>
#include <mutex>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

template <typename T>
using cloud = pcl::PointCloud<T>;

namespace climbing_robot
{

class Mapping
{
public:
    Mapping(ros::NodeHandle node);
    ~Mapping(){}

    void CalculateTransform(const std::vector<double>& sixDof, Eigen::Isometry3d& transMat);
    void CalculateTransform(const Eigen::Quaterniond& quat, const Eigen::Vector3d& vec3d,
                                Eigen::Isometry3d& transMat);

    // void Match(const cloud<pcl::PointXYZ>::Ptr cloud_ptr, const Eigen::Isometry3d& pose_mat);
    void Match();

    void removeClosedPointCloud(const cloud<pcl::PointXYZ> &cloud_in,
                            cloud<pcl::PointXYZ> &cloud_out, double minThres, double maxThres);


private:

    void PointCloudHandler(const sensor_msgs::PointCloud2ConstPtr &CloudMsg);
    void PoseHandler(const geometry_msgs::PoseStampedConstPtr &PoseMsg);


    cloud<pcl::PointXYZ>::Ptr gt_map_;
    cloud<pcl::PointXYZ>::Ptr syn_cloud_;
    cloud<pcl::PointXYZ>::Ptr transformed_cloud_;
    geometry_msgs::PoseStamped syn_pose_;
    

    std::queue<sensor_msgs::PointCloud2ConstPtr> cloud_queue_;
    std::queue<geometry_msgs::PoseStamped> pose_queue_;

    double min_range_;
    double max_range_;
    std::string cloud_topic_;
    std::string pose_topic_;

    std::mutex mutex_buf_;

    ros::Subscriber sub_pointcloud_;
    ros::Subscriber sub_pose_;

    ros::Publisher pub_map_;

};


}//namespcae climbing_robot


#endif//CLIMBING_ROBOT_GT_MAP