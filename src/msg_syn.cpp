#include "msg_syn.h"

#include <iostream>

namespace climbing_robot
{


MsgSynchronizer::MsgSynchronizer(ros::NodeHandle node)
{
    pub_cloud_ = node.advertise<sensor_msgs::PointCloud2>("/Syn/cloud", 1);
    pub_pose_ = node.advertise<geometry_msgs::PoseStamped>("/Syn/pose", 1);

    node.getParam("msg_syn_bag_path", syn_bag_path);
    msg_syn_bag.open(syn_bag_path, rosbag::bagmode::Write);
}


void MsgSynchronizer::MessageFilter(ros::NodeHandle node)
{
    message_filters::Subscriber<geometry_msgs::PoseStamped> pose_sub(node, "/vrpn_client_node/lidar/pose", 100);
    message_filters::Subscriber<sensor_msgs::PointCloud2> velodyne_sub(node, "/velodyne_points", 10);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, geometry_msgs::PoseStamped> approximate_policy;
    message_filters::Synchronizer<approximate_policy> sync(approximate_policy(4), velodyne_sub, pose_sub);
    sync.registerCallback(boost::bind(&MsgSynchronizer::callback, this, _1, _2));

    ros::spin();
}


void MsgSynchronizer::callback(const sensor_msgs::PointCloud2::ConstPtr& ori_pointcloud, const geometry_msgs::PoseStamped::ConstPtr& ori_pose)
{
    // ROS_INFO("pointcloud stamp value is: %f", ori_pointcloud->header.stamp.toSec());
    // ROS_INFO("pose stamp value is: %f", ori_pose->header.stamp.toSec());
   
    msg_syn_bag.write("/velodyne_points", ori_pointcloud->header.stamp, *ori_pointcloud);
    msg_syn_bag.write("/vrpn_client_node/lidar/pose", ori_pose->header.stamp, *ori_pose);

    pub_cloud_.publish(*ori_pointcloud);
    pub_pose_.publish(*ori_pose);
}

}//namespace climbing_robot


int main(int argc, char** argv)
{
    ros::init(argc, argv, "msg_synchronizer");
    ros::NodeHandle node("~");
    ROS_INFO("\033[1;32m---->\033[0m Sync msgs node Started.");

    climbing_robot::MsgSynchronizer synchronizer(node);

    synchronizer.MessageFilter(node);
    // ros::spin();

    synchronizer.CloseBag();

    return 0;
}