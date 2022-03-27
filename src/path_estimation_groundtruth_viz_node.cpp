#include <ros/ros.h>
#include "path_estimation_groundtruth_viz/PathVisualizer.hpp"

using namespace path_estimation_groundtruth_viz;

int main(int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "path_estimation_groundtruth_viz");
    ros::NodeHandle n;

    // PathVisualizer estimated_path_visualizer();
    // PathVisualizer ground_truth_visualizer();

    ROS_INFO("hello from path_compare_viz.");

    ros::spin();

    return 0;
}