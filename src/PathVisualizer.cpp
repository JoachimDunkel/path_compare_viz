#include "path_compare_viz/PathVisualizer.hpp"
#include <nav_msgs/Path.h>

namespace path_compare_viz
{

PathVisualizer::PathVisualizer(   
    const std::string & tag,         
    SOURCE_TYPE source_type,
    const std::string &pose_source_topic,
    const std::string &path_target_topic,
    ros::NodeHandle& handle) 
    : tag_(tag),
      poseSourceTopic_(pose_source_topic),
      pathTargetTopic_(path_target_topic),
      nodeHandle_(handle)
{
    pathPublisher_ = nodeHandle_.advertise<nav_msgs::Path>(path_target_topic, 1);
    if(source_type == SOURCE_TYPE::POSE){
        poseSubscriber_ = nodeHandle_
            .subscribe(pose_source_topic, 1, &PathVisualizer::poseCallback, this);
    }
    else{
        poseStampedSubscriber_ = nodeHandle_
            .subscribe(pose_source_topic, 1, &PathVisualizer::poseStampedCallback, this);
    }

    ROS_INFO("%s Started.", tag_.c_str());
}

//This is not pose stamped since gazebo ground truths will be published as Pose
void PathVisualizer::poseCallback(const geometry_msgs::Pose& msg) 
{
    ROS_INFO("%s Recieved pose callback", tag_.c_str());
    //promote to poseStamped (take current time-stamp)

    //if the current pose does not diverge enough from the last pose ignore callback

    //possible also ignore msgs that are to close to each other on a time basis if to many msgs 
    //start existing.

    //when we get a message store it in vector
    //send path.
}

void PathVisualizer::poseStampedCallback(const geometry_msgs::PoseStamped& msg)
{
    ROS_INFO("%s Recieved poseStamped callback", tag_.c_str());

}


} 


