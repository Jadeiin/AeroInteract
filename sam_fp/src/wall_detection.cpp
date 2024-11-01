#include "wall_detection/wall_detection.h"

void wall_detection::cloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg) {
  pcl::fromROSMsg(*msg, *raw_cloud_);
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients(true);
  // Mandatory
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(0.01);
  seg.setInputCloud(raw_cloud_);
  seg.segment(*inliers, *coefficients);
  if (inliers->indices.size() == 0) {
    ROS_ERROR("Could not estimate a planar model for the given dataset.");
    return;
  }
  if (inliers->indices.size() < 20000) {
    ROS_ERROR("Could not estimate a planar model for the given dataset.");
    return;
  }
  pcl::copyPointCloud(*raw_cloud_, inliers->indices, *wall_cloud_);
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(*wall_cloud_, centroid);
  pcl::toROSMsg(*wall_cloud_, cloudmsg_);
  wall_points_pub_.publish(cloudmsg_);

  // Publish arrow
  visualization_msgs::Marker marker;
  marker.header.frame_id = "camera_color_optical_frame";
  marker.header.stamp = ros::Time::now();
  marker.ns = "wall_arrow";
  marker.type = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::ADD;
  Eigen::Vector3f start(centroid[0], centroid[1], centroid[2]);
  Eigen::Vector3f end(centroid[0] + coefficients[0],
                      centroid[1] + coefficients[1],
                      centroid[2] + coefficients[2]);
  marker.points.push_back(start);
  marker.points.push_back(end);
  marker.color.r = 0.0f;
  marker.color.g = 0.0f;
  marker.color.b = 1.0f;
  marker.color.a = 1.0;
  wall_marker_pub_.publish(marker);

  return;
}