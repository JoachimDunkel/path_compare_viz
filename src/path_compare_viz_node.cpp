#include <ros/ros.h>
#include "path_compare_viz/PathVisualizer.hpp"

using namespace path_compare_viz;

int main(int argc, char** argv)
{
    ros::init (argc, argv, "path_compare_viz");
    ros::NodeHandle n;

    //TODO make source type configurable form launch file
    //make source and target configurable from launch file

    PathVisualizer estimated_path_visualizer("[ESTIMATION VISUALIZER]", SOURCE_TYPE::POSESTAMPED , "base_link_pf", "base_link_pf_path", n);
    PathVisualizer ground_truth_visualizer("[GROUND_TRUTH VISUALIZER]", SOURCE_TYPE::POSESTAMPED, "base_link_real", "base_link_real_path", n);

    //TODO possibly we only want to send data once on shut down hook or something.s

    ros::spin();

    return 0;
}