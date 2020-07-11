#ifndef CLIMBING_ROBOT_GT_MAP_
#define CLIMBING_ROBOT_GT_MAP_

#include <string>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

namespace climbing_robot
{


class MsgSynchronizer{
public:
    MsgSynchronizer(ros::NodeHandle node);
    ~MsgSynchronizer(){};

    void MessageFilter(ros::NodeHandle node);
    void callback(const sensor_msgs::PointCloud2::ConstPtr& ori_pointcloud, const geometry_msgs::PoseStamped::ConstPtr& ori_pose);
    
    inline void CloseBag()
    {
        msg_syn_bag.close();
    }

private:
    ros::Publisher pub_cloud_;
    ros::Publisher pub_pose_;
    std::string syn_bag_path;
    rosbag::Bag msg_syn_bag;
};



}//namespace climbing_robot


#endif// CLIMBING_ROBOT_GT_MAP_

