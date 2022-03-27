#include <ros/ros.h>
#include "path_compare_viz/PathVisualizer.hpp"

using namespace path_compare_viz;

int main(int argc, char** argv)
{
    ros::init (argc, argv, "path_compare_viz");
    ros::NodeHandle n;

    //TODO make source type configurable form launch file
    //make source and target configurable from launch file
    std::string estimate_source, estimate_target, ground_truth_source, ground_truth_target;
    n.getParam("topic_estimated_source_pose", estimate_source);
    n.getParam("topic_estimated_target_path", estimate_target);

    n.getParam("topic_ground_truth_source_pose", ground_truth_source);
    n.getParam("topic_ground_truth_target_path", ground_truth_target);

    int estimation_source_type, ground_truth_source_type;
    n.getParam("estimation_source_type", estimation_source_type);
    n.getParam("ground_truth_source_type", ground_truth_source_type);

    PathVisualizer estimated_path_visualizer("[ESTIMATION VISUALIZER]", static_cast<SOURCE_TYPE>(estimation_source_type) , estimate_source, estimate_target, n);
    PathVisualizer ground_truth_visualizer("[GROUND_TRUTH VISUALIZER]", static_cast<SOURCE_TYPE>(ground_truth_source_type), ground_truth_source, ground_truth_target, n);

    //TODO possibly we only want to send data once on shut down hook or something.s

    ros::spin();

    return 0;
}