#include "wall_detection/wall_detection.h"

int main(int argc, char **argv) {
  // Initialize ROS
  ros::init(argc, argv, "wall_detection_node");

  wall_detection wall_detection_obj;
  ros::spin();
  return 0;
}