#include "path_compare_viz/PathVisualizer.hpp"
#include <nav_msgs/Path.h>
#include <math.h>
#include <tf/tf.h>

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
    //ROS_INFO("%s Recieved poseStamped callback", tag_.c_str());

    if(storedPoses_.size() < 1){
        storedPoses_.push_back(msg);
        return;
    }

    auto last_stored_pose = storedPoses_.back();
    if(!enoughDifference(msg.pose, last_stored_pose.pose)){
        return;
    }

    storedPoses_.push_back(msg);

    nav_msgs::Path path;
    path.header.frame_id = pathTargetTopic_;
    path.header.stamp = ros::Time(0);

    path.poses = storedPoses_;

    pathPublisher_.publish(path);
}

double angleDiff(const double & a, const double & b) 
{
    return atan2(sin(a - b), cos(a - b));  //see  https://stackoverflow.com/questions/1878907/how-can-i-find-the-difference-between-two-angles
}

bool PathVisualizer::enoughDifference(const geometry_msgs::Pose& currentPose, const geometry_msgs::Pose& lastPose) 
{
    double dist_thresh = 0.001; //1 mm
    double angle_thresh = 0.002; // 1°

    double x_diff = fabs(currentPose.position.x) - fabs(lastPose.position.x);
    double y_diff = fabs(currentPose.position.y) - fabs(lastPose.position.y);

    double theta_diff = angleDiff(tf::getYaw(currentPose.orientation), tf::getYaw(lastPose.orientation));

    return (x_diff > dist_thresh || y_diff > dist_thresh || theta_diff > angle_thresh);
}


} 


