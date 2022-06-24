#include <ros/ros.h>
#include "path_compare_viz/PathVisualizer.hpp"

using namespace path_compare_viz;

int main(int argc, char** argv)
{
    ros::init (argc, argv, "single_path_visualizer");
    ros::NodeHandle n;

    std::string source_pose, target_path;
    n.getParam("source_pose", source_pose);
    n.getParam("target_path", target_path);

    int source_type;
    n.getParam("source_type", source_type);

	std::string frame_id;
	n.getParam("frame_id_to_publish", frame_id);

	double dist_thresh, angle_thresh;
	n.getParam("dist_threshold", dist_thresh);
	n.getParam("angle_threshold", angle_thresh);

    PathVisualizer estimated_path_visualizer("[SINGLE PATH VISUALIZER]", frame_id,
		 static_cast<SOURCE_TYPE>(source_type) , source_pose, target_path,
		 dist_thresh, angle_thresh, n);


    ros::spin();

    return 0;
}