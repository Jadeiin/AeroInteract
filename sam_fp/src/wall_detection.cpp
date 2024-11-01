#include "wall_detection/wall_detection.h"

void wall_detection::cloudCallback(
    const sensor_msgs::PointCloud2ConstPtr &msg) {
  pcl::fromROSMsg(*msg, *raw_cloud_);
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<point> seg;
  // Optional
  seg.setOptimizeCoefficients(true);
  // Mandatory
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(0.01);
  seg.setInputCloud(raw_cloud_);
  seg.segment(*inliers, *coefficients);
  if (inliers->indices.size() == 0) {
    ROS_ERROR(
        "Could not estimate a planar model for the given dataset (size = 0).");
    return;
  }
  if (inliers->indices.size() < 20000) {
    ROS_ERROR(
        "Could not estimate a planar model for the given dataset (size too "
        "small).");
    return;
  }
  pcl::copyPointCloud(*raw_cloud_, inliers->indices, *wall_cloud_);
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(*wall_cloud_, centroid);
  pcl::toROSMsg(*wall_cloud_, cloudmsg_);
  wall_points_pub_.publish(cloudmsg_);

  // Publish arrow
  wall_marker_.header.frame_id = "camera_color_optical_frame";
  wall_marker_.header.stamp = ros::Time::now();
  wall_marker_.ns = "wall_arrow";
  wall_marker_.type = visualization_msgs::Marker::ARROW;
  wall_marker_.action = visualization_msgs::Marker::ADD;
  geometry_msgs::Point start;
  start.x = centroid[0];
  start.y = centroid[1];
  start.z = centroid[2];
  geometry_msgs::Point end;
  end.x = centroid[0] + coefficients->values[0];
  end.y = centroid[1] + coefficients->values[1];
  end.z = centroid[2] + coefficients->values[2];
  wall_marker_.points.push_back(start);
  wall_marker_.points.push_back(end);
  wall_marker_.color.r = 0.0f;
  wall_marker_.color.g = 0.0f;
  wall_marker_.color.b = 1.0f;
  wall_marker_.color.a = 1.0;
  wall_marker_.scale.x = 0.1;
  wall_marker_.scale.y = 0.1;
  wall_marker_.scale.z = 0.1;
  wall_marker_pub_.publish(wall_marker_);

  return;
}

void wall_detection::obbCallback(
    const visualization_msgs::MarkerArrayConstPtr &msg) {
  // TODO: Handle multiple OBB
  visualization_msgs::Marker bbox_marker = msg->markers[0];  // Only one OBB
  Eigen::Vector3f bbox_centroid(bbox_marker.pose.position.x,
                                bbox_marker.pose.position.y,
                                bbox_marker.pose.position.z);
  Eigen::Quaternionf bbox_quat(
      bbox_marker.pose.orientation.w, bbox_marker.pose.orientation.x,
      bbox_marker.pose.orientation.y, bbox_marker.pose.orientation.z);
  Eigen::Vector3f bbox_vec =
      bbox_quat._transformVector(Eigen::Vector3f::UnitX());
  Eigen::Vector3f wall_centroid(wall_marker_.points[0].x,
                                wall_marker_.points[0].y,
                                wall_marker_.points[0].z);
  Eigen::Vector3f wall_end(wall_marker_.points[1].x, wall_marker_.points[1].y,
                           wall_marker_.points[1].z);
  Eigen::Vector3f wall_vec = wall_end - wall_centroid;
  // // method 1: directly calculate angle between two vectors (suppose wall and
  // object are) double angle = std::acos(wall_vec.dot(bbox_vec) /
  // (wall_vec.norm() * bbox_vec.norm()));
  // // make sure the angle is in the range of [0, pi/2]
  // if (angle > M_PI / 2) {
  //     angle = M_PI - angle;
  // }
  // // method 2: calcualte euler angles seperately and then combine
  // Eigen::Matrix3f bbox_rotation_matrix = bbox_quat.toRotationMatrix();
  // Eigen::Vector3f bbox_euler_angles = rotation_matrix.eulerAngles(2, 1, 0);
  // method 3: calculate quaternion from two vectors
  Eigen::Vector3f euler_angles =
      Eigen::Quaternionf::FromTwoVectors(wall_vec, bbox_vec)
          .toRotationMatrix()
          .eulerAngles(2, 1, 0);
  double angle = euler_angles[2];
  if (angle > M_PI / 2) {
    angle = angle - M_PI / 2;
  }

  object_angle_.header.frame_id = "camera_color_optical_frame";
  object_angle_.header.stamp = ros::Time::now();
  object_angle_.ns = "object_angle";
  object_angle_.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  object_angle_.action = visualization_msgs::Marker::ADD;
  object_angle_.pose.position.x = bbox_centroid[0];
  object_angle_.pose.position.y = bbox_centroid[1];
  object_angle_.pose.position.z = bbox_centroid[2];
  object_angle_.color.r = 1.0f;
  object_angle_.color.g = 1.0f;
  object_angle_.color.b = 1.0f;
  object_angle_.color.a = 1.0;
  std::stringstream ss;
  ss << std::fixed << std::setprecision(2) << (angle * 180 / M_PI);
  object_angle_.text = ss.str() + "°";
  object_angle_.scale.z = 0.5;
  object_angle_pub_.publish(object_angle_);
  return;
}