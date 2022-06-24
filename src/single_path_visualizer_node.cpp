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

    PathVisualizer estimated_path_visualizer(frame_id, "[SINGLE PATH VISUALIZER]",
		 static_cast<SOURCE_TYPE>(source_type) , source_pose, target_path, n);


    ros::spin();

    return 0;
}