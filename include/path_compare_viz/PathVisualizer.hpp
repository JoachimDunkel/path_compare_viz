#pragma once

#include <ros/ros.h>
#include <string>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>

namespace path_compare_viz
{  
enum SOURCE_TYPE{
    POSE = 0,
    POSESTAMPED = 1,
    ODOMETRY = 2
};

class PathVisualizer{

    public:
        explicit PathVisualizer(
            const std::string & tag,
			const std::string frame_id_to_publish,
            SOURCE_TYPE source_type,
            const std::string &pose_source_topic,
            const std::string &path_target_topic,
			const double dist_threshold,
			const double angle_threshold,
            ros::NodeHandle& handle);

        void poseCallback(const geometry_msgs::Pose& msg);
        void poseStampedCallback(const geometry_msgs::PoseStamped& msg);
        void odomCallback(const nav_msgs::Odometry& msg);

    private:
        void processPose(const geometry_msgs::PoseStamped& pose);
        geometry_msgs::PoseStamped promotePose(const geometry_msgs::Pose& pose);
        bool enoughDifference(const geometry_msgs::Pose& currentPose, const geometry_msgs::Pose& lastPose);

        const std::string tag_;
		const std::string frame_id_to_publish_;
        const std::string poseSourceTopic_;
        const std::string pathTargetTopic_;

		const double dist_threshold_;
		const double angle_threshold_;

        ros::NodeHandle & nodeHandle_;
        ros::Subscriber poseSubscriber_;
        ros::Publisher pathPublisher_;

        std::vector<geometry_msgs::PoseStamped> storedPoses_;
};
}

