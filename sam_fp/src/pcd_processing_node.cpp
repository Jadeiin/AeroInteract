#include "pcd_processing/pcd_processing.h"

int main(int argc, char **argv) {
    // Initialize ROS
    ros::init(argc, argv, "pcd_processing_node");
    ros::NodeHandle nh;
    
    // Create the pcd_processing object`
    pcd_processing pcd_processing_obj;
    // Initialize the pcd_processing object
    if (!pcd_processing_obj.initialize(nh)) {
        ROS_ERROR("Failed to initialize the pcd_processing object!");
        return -1;
    }

    // Set the loop rate in Hz
    ros::Rate loop_rate(30);
    // Main loop
    while (ros::ok()) {
        // Update the pcd_processing object
        pcd_processing_obj.update(ros::Time::now());

        // Spin and sleep
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}   