#include "pcd_processing/pcd_processing.h"

int main(int argc, char **argv) {
  // Initialize ROS
  ros::init(argc, argv, "pcd_processing_node");

  // Create node handle
  ros::NodeHandle nh("~");

  // Create and initialize the pcd_processing object
  pcd_processing pcd_processing_obj(nh);

  // Set the loop rate in Hz
  ros::Rate loop_rate(30);
  
  // Main loop
  // TODO: make it in real-time
  while (ros::ok()) {
    // Update the pcd_processing object
    pcd_processing_obj.update(ros::Time::now());

    // Spin and sleep
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}