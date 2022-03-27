#include "path_compare_viz/PathVisualizer.hpp"

namespace path_compare_viz
{

PathVisualizer::PathVisualizer(
    const std::string &pose_source_topic,
    const std::string &path_target_topic,
    ros::NodeHandle& handle) : nodeHandle_(handle)
{
    //setup subscriber and publisher.
}

void PathVisualizer::poseCallback(const geometry_msgs::PoseStamped& msg) 
{
    

    //if the current pose does not diverge enough from the last pose ignore callback

    //possible also ignore msgs that are to close to each other on a time basis if to many msgs 
    //start existing.

    //when we get a message store it in vector
    //send path.
}

} 


