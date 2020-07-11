#include "mapping.h"

#include <boost/make_shared.hpp>
#include <cmath>

namespace climbing_robot
{


Mapping::Mapping(ros::NodeHandle nh)
{
    
    nh.getParam("min_range", min_range_);
    nh.getParam("max_range", max_range_);
    nh.getParam("cloud_topic", cloud_topic_);
    nh.getParam("pose_topic", pose_topic_);
    
    sub_pointcloud_ = nh.subscribe<sensor_msgs::PointCloud2>(cloud_topic_, 10, &Mapping::PointCloudHandler, this);
    sub_pose_ = nh.subscribe<geometry_msgs::PoseStamped>(pose_topic_, 100, &Mapping::PoseHandler, this);

    pub_map_ = nh.advertise<sensor_msgs::PointCloud2>("/climbing_robot/map", 1);
    
    gt_map_ = boost::make_shared <pcl::PointCloud<pcl::PointXYZ> > ();
    syn_cloud_ = boost::make_shared <pcl::PointCloud<pcl::PointXYZ> > ();
    transformed_cloud_ = boost::make_shared <pcl::PointCloud<pcl::PointXYZ> > ();


    if(cloud_queue_.empty() && pose_queue_.empty() )
        std::cout << "Initialize successfully! \n";
}


void Mapping::removeClosedPointCloud(const cloud<pcl::PointXYZ> &cloud_in,
                            cloud<pcl::PointXYZ> &cloud_out, double minThres, double maxThres)
{
    if (&cloud_in != &cloud_out)
    {
        cloud_out.header = cloud_in.header;
        cloud_out.points.resize(cloud_in.points.size());
    }

    size_t j = 0;

    for (size_t i = 0; i < cloud_in.points.size(); ++i)
    {
        float dist = cloud_in.points[i].x * cloud_in.points[i].x + cloud_in.points[i].y * cloud_in.points[i].y + cloud_in.points[i].z * cloud_in.points[i].z;
        if (dist < minThres * minThres || dist > maxThres * maxThres)
            continue;
        cloud_out.points[j] = cloud_in.points[i];
        j++;
    }
    if (j != cloud_in.points.size())
    {
        cloud_out.points.resize(j);
    }

    cloud_out.height = 1;
    cloud_out.width = static_cast<uint32_t>(j);
    cloud_out.is_dense = true;
}


void Mapping::PointCloudHandler(const sensor_msgs::PointCloud2ConstPtr &CloudMsg)
{
    mutex_buf_.lock();
    cloud_queue_.push(CloudMsg);
    mutex_buf_.unlock();
}


void Mapping::PoseHandler(const geometry_msgs::PoseStampedConstPtr &PoseMsg)
{
    mutex_buf_.lock();
    pose_queue_.push(*PoseMsg);
    mutex_buf_.unlock();
}


void Mapping::Match()
{
    if(cloud_queue_.size() < 1 || pose_queue_.size() < 1)
        return;


    mutex_buf_.lock();
    
    sensor_msgs::PointCloud2ConstPtr cloud_ptr = cloud_queue_.front();
    cloud_queue_.pop();
    geometry_msgs::PoseStamped pose = pose_queue_.front();
    pose_queue_.pop();

    mutex_buf_.unlock();

    std_msgs::Header cloud_header = (*cloud_ptr).header;
    std_msgs::Header pose_header = pose.header;

    if(std::abs(cloud_header.stamp.nsec - pose_header.stamp.nsec) > 100000000) // 时间戳差值大于0.1s
    {
        std::cout << "Synchronization fail, drop this pair of message! \n";
        return;
    }
    
    syn_cloud_->clear();
    pcl::fromROSMsg(*cloud_ptr, *syn_cloud_);

    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*syn_cloud_, *syn_cloud_, indices);
    Mapping::removeClosedPointCloud(*syn_cloud_, *syn_cloud_, min_range_, max_range_);


    std::cout << "------point cloud------- : ";
    ROS_INFO("pointcloud stamp value is: %f", cloud_header.stamp.toSec());
    std::cout << "pose : ";
    ROS_INFO("pose stamp value is: %f", pose_header.stamp.toSec());


    Eigen::Quaterniond quat(pose.pose.orientation.w, pose.pose.orientation.x, 
                            pose.pose.orientation.y, pose.pose.orientation.z);
    Eigen::Vector3d displacement(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);

    std::cout << "displacement = " << displacement << std::endl;

    Eigen::Isometry3d pose_mat;
    CalculateTransform(quat, displacement, pose_mat);

    Eigen::Matrix4d relative_trans = pose_mat.matrix();// relative_trans 是在地图坐标系中描述sensor的位姿

    std::cout << "relative_trans = " << relative_trans << std::endl;

    transformed_cloud_->clear();
    pcl::transformPointCloud(*syn_cloud_, *transformed_cloud_, relative_trans);
    // pcl::transformPointCloud(const pcl::PointCloud< PointT > & cloud_in, 
    //                          pcl::PointCloud< PointT > &  cloud_out,  
    //                            const Eigen::Matrix4f &  transform  )


    *gt_map_ = *gt_map_ + *transformed_cloud_;
    std::cout << "map size : " << gt_map_->size() << std::endl;

    cloud<pcl::PointXYZ>::Ptr filtered_map (new cloud<pcl::PointXYZ>);
    pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
    approximate_voxel_filter.setLeafSize (0.05, 0.05, 0.05);
    approximate_voxel_filter.setInputCloud (gt_map_);
    approximate_voxel_filter.filter (*filtered_map);
    std::cout << "Filtered map contains " << filtered_map->size() << std::endl;
    
    sensor_msgs::PointCloud2 map_cloud;
    pcl::toROSMsg(*filtered_map, map_cloud);
    map_cloud.header.stamp = ros::Time::now();
    map_cloud.header.frame_id = "/world";
    pub_map_.publish(map_cloud);
}


void Mapping::CalculateTransform(const Eigen::Quaterniond& quat, const Eigen::Vector3d& vec3d,
                                Eigen::Isometry3d& transMat)
{
    Eigen::Isometry3d rot;
    rot.matrix().block<3,3>(0,0) = quat.matrix();
    Eigen::Translation3d translation( vec3d(0), vec3d(1), vec3d(2) );
    transMat = translation*rot;

    std::cout << "translation : " << translation.translation() << std::endl;
}


// void calculateTransform(const std::vector<double>& sixDof, Eigen::Isometry3d& transMat)
// {
//     Eigen::AngleAxisd xRot(sixDof[3]/180.0*M_PI, Eigen::Vector3d::UnitX());
//     Eigen::AngleAxisd yRot(sixDof[4]/180.0*M_PI, Eigen::Vector3d::UnitY());
//     Eigen::AngleAxisd zRot(sixDof[5]/180.0*M_PI, Eigen::Vector3d::UnitZ());
//     Eigen::Translation3d translation(sixDof[0], sixDof[1], sixDof[2]);
//     transMat = (translation*zRot*yRot*xRot).matrix();
// }


}//namespace climbing_robot




int main(int argc, char** argv)
{
    ros::init(argc, argv, "mapping");
    ros::NodeHandle node("~");
    ROS_INFO("\033[1;32m---->\033[0m mapping node Started.");

    climbing_robot::Mapping map(node);

    ros::Rate rate(10);

    while (ros::ok())
    {
        ros::spinOnce();

        map.Match();

        rate.sleep();
    }

    return 0;
}


