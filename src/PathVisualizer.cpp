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

    switch (source_type)
    {
    case POSE:
        poseSubscriber_ = nodeHandle_
            .subscribe(pose_source_topic, 1, &PathVisualizer::poseCallback, this);
        break;

    case POSESTAMPED:
        poseSubscriber_ = nodeHandle_
            .subscribe(pose_source_topic, 1, &PathVisualizer::poseStampedCallback, this);
        break;

    case ODOMETRY:
        poseSubscriber_ = nodeHandle_
            .subscribe(pose_source_topic, 1, &PathVisualizer::odomCallback, this);
        break;
    
    default:
        ROS_ERROR("Unhandled source_type, shutting down.");
        ros::requestShutdown();
    }
    ROS_INFO("%s Started.", tag_.c_str());
}


void PathVisualizer::poseCallback(const geometry_msgs::Pose& msg) 
{
    auto current_pose = promotePose(msg);
    processPose(current_pose);
}

void PathVisualizer::poseStampedCallback(const geometry_msgs::PoseStamped& msg)
{
    processPose(msg);
}

void PathVisualizer::odomCallback(const nav_msgs::Odometry& msg)
{
    auto current_pose = promotePose(msg.pose.pose);
    processPose(current_pose);
}

void PathVisualizer::processPose(const geometry_msgs::PoseStamped& pose) 
{
    if(storedPoses_.size() < 1){
        storedPoses_.push_back(pose);
        return;
    }

    auto last_stored_pose = storedPoses_.back();
    if(!enoughDifference(pose.pose, last_stored_pose.pose)){
        return;
    }

    storedPoses_.push_back(pose);

    nav_msgs::Path path;
    path.header.frame_id = "map";
    path.header.stamp = ros::Time(0);

    path.poses = storedPoses_;

    pathPublisher_.publish(path);
}

double angleDiff(const double & a, const double & b) 
{
    return atan2(sin(a - b), cos(a - b));  //see  https://stackoverflow.com/questions/1878907/how-can-i-find-the-difference-between-two-angles
}

geometry_msgs::PoseStamped PathVisualizer::promotePose(const geometry_msgs::Pose& pose)
{
    geometry_msgs::PoseStamped current_pose;
    current_pose.header.frame_id = "map";
    current_pose.header.stamp = ros::Time(0);

    current_pose.pose = pose;
    return current_pose;
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


