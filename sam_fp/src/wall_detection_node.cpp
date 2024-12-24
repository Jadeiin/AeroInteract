#include "wall_detection/wall_detection.h"

int main(int argc, char **argv) {
  // Initialize ROS
  ros::init(argc, argv, "wall_detection_node");

  // Create node handle
  ros::NodeHandle nh;

  // Create and initialize the wall_detection object
  wall_detection wall_detection_obj(nh);

  ros::spin();
  return 0;
}