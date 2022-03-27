#pragma once

#include <ros/ros.h>
#include <string>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>

namespace path_compare_viz
{  
enum SOURCE_TYPE{
    POSE = 0,
    POSESTAMPED = 1,
};

class PathVisualizer{

    public:
        explicit PathVisualizer(
            const std::string & tag,
            SOURCE_TYPE source_type,
            const std::string &pose_source_topic,
            const std::string &path_target_topic,
            ros::NodeHandle& handle);

        void poseCallback(const geometry_msgs::Pose& msg);
        void poseStampedCallback(const geometry_msgs::PoseStamped& msg);

    private:
        bool enoughDifference(const geometry_msgs::Pose& currentPose, const geometry_msgs::Pose& lastPose);

        const std::string tag_;
        const std::string poseSourceTopic_;
        const std::string pathTargetTopic_;

        ros::NodeHandle & nodeHandle_;
        ros::Subscriber poseSubscriber_;
        ros::Subscriber poseStampedSubscriber_;

        ros::Publisher pathPublisher_;

        std::vector<geometry_msgs::PoseStamped> storedPoses_;
};
}

