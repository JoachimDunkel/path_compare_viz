#pragma once

#include <ros/ros.h>
#include <string>
#include <geometry_msgs/PoseStamped.h>

namespace path_estimation_groundtruth_viz
{  
class PathVisualizer{

    public:
        explicit PathVisualizer(
            const std::string &pose_source_topic,
            const std::string &path_target_topic,
            ros::NodeHandle& handle);

        void poseCallback(const geometry_msgs::PoseStamped& msg);

    private:
        std::string const tag_ = "[PATH_VISUALIZER] ";
        ros::NodeHandle & nodeHandle_;
        ros::Subscriber poseSubscriber_;
        ros::Publisher pathPublisher_;

        std::vector<geometry_msgs::PoseStamped> storedPoses_;
};
}

