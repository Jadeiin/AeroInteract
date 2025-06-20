#include "wall_detection/wall_detection.h"

// Constructor
wall_detection::wall_detection(ros::NodeHandle &nh_) : nh(nh_) {
  // Get parameters from the parameter server
  nh.param<std::string>("pointcloud_topic", pointcloud_topic_,
                        "/background_cloud");
  nh.param<std::string>("base_frame", base_frame_, "camera_link");
  nh.param<bool>("/enable_metrics", enable_metrics_, false);

  // Setup subscribers and publishers
  point_cloud_sub_ =
      nh.subscribe(pointcloud_topic_, 10, &wall_detection::cloudCallback, this);
  objects_marker_sub_ = nh.subscribe("/objects_marker", 10,
                                     &wall_detection::objectCallback, this);
  wall_points_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/wall_points", 10);
  wall_marker_pub_ =
      nh.advertise<visualization_msgs::Marker>("/wall_marker", 10);
  object_angle_pub_ =
      nh.advertise<visualization_msgs::Marker>("/object_angle", 10);

  // Initialize pointers
  raw_cloud_.reset(new cloud);
  wall_cloud_.reset(new cloud);
}

void wall_detection::cloudCallback(
    const sensor_msgs::PointCloud2ConstPtr &msg) {
  pcl::fromROSMsg(*msg, *raw_cloud_);
  if (raw_cloud_->size() == 0) {
    ROS_ERROR("Empty pointcloud received.");
    return;
  }
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
  if (inliers->indices.size() < 1500) {
    ROS_ERROR(
        "Could not estimate a planar model for the given dataset (size = %ld).",
        inliers->indices.size());
    return;
  }
  // TODO: check if the wall contains the object
  pcl::copyPointCloud(*raw_cloud_, inliers->indices, *wall_cloud_);
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(*wall_cloud_, centroid);
  pcl::toROSMsg(*wall_cloud_, cloudmsg_);
  wall_points_pub_.publish(cloudmsg_);

  // Transform the start and end points
  geometry_msgs::PointStamped start_base, start_cam, end_base;
  geometry_msgs::Vector3Stamped normal_base, normal_cam;
  start_cam.header.frame_id = normal_cam.header.frame_id = msg->header.frame_id;
  start_cam.point.x = centroid[0];
  start_cam.point.y = centroid[1];
  start_cam.point.z = centroid[2];
  normal_cam.vector.x = coefficients->values[0];
  normal_cam.vector.y = coefficients->values[1];
  normal_cam.vector.z = coefficients->values[2];
  try {
    tf_listener_.transformPoint(base_frame_, start_cam, start_base);
    tf_listener_.transformVector(base_frame_, normal_cam, normal_base);
  } catch (tf::TransformException &ex) {
    ROS_ERROR("%s", ex.what());
    return;
  }
  end_base.point.x = start_base.point.x + normal_base.vector.x;
  end_base.point.y = start_base.point.y + normal_base.vector.y;
  end_base.point.z = start_base.point.z + normal_base.vector.z;

  // Publish arrow
  wall_marker_.points.clear();
  wall_marker_.header.frame_id = base_frame_;
  wall_marker_.header.stamp = ros::Time::now();
  wall_marker_.ns = "wall_arrow";
  wall_marker_.type = visualization_msgs::Marker::ARROW;
  wall_marker_.action = visualization_msgs::Marker::ADD;
  wall_marker_.points.push_back(start_base.point);
  wall_marker_.points.push_back(end_base.point);
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

void wall_detection::objectCallback(
    const visualization_msgs::MarkerArrayConstPtr &msg) {
  // TODO: Handle multiple OBB
  // ROS_INFO("objectCallback is triggered.");
  if (wall_marker_.points.size() == 0) {
    ROS_ERROR("Wall marker not produced.");
    return;
  }
  visualization_msgs::Marker object_marker = msg->markers[0];  // Only one OBB
  Eigen::Vector3f object_centroid(object_marker.points[0].x,
                                  object_marker.points[0].y,
                                  object_marker.points[0].z);
  Eigen::Vector3f object_end(object_marker.points[1].x,
                             object_marker.points[1].y,
                             object_marker.points[1].z);
  Eigen::Vector3f object_vec = object_end - object_centroid;
  Eigen::Vector3f wall_centroid(wall_marker_.points[0].x,
                                wall_marker_.points[0].y,
                                wall_marker_.points[0].z);
  Eigen::Vector3f wall_end(wall_marker_.points[1].x, wall_marker_.points[1].y,
                           wall_marker_.points[1].z);
  Eigen::Vector3f wall_vec = wall_end - wall_centroid;
  // make sure two vectors are directed into same way
  if (wall_vec.dot(object_vec) < 0) {
    object_vec = -object_vec;
  }
  // // method 1: directly calculate angle between two vectors (suppose wall and
  // object are) double angle = std::acos(wall_vec.dot(object_vec) /
  // (wall_vec.norm() * object_vec.norm()));
  // // make sure the angle is in the range of [0, pi/2]
  // if (angle > M_PI / 2) {
  //     angle = M_PI - angle;
  // }
  // // method 2: calcualte euler angles seperately and then combine
  // Eigen::Matrix3f bbox_rotation_matrix = bbox_quat.toRotationMatrix();
  // Eigen::Vector3f bbox_euler_angles = rotation_matrix.eulerAngles(2, 1, 0);
  // method 3: calculate quaternion from two vectors
  Eigen::Vector3f euler_angles =
      Eigen::Quaternionf::FromTwoVectors(wall_vec, object_vec)
          .toRotationMatrix()
          .eulerAngles(2, 1, 0);
  if (enable_metrics_) {
    ROS_INFO_STREAM("Wall vector: " << wall_vec);
    ROS_INFO_STREAM("Object vector: " << object_vec);
    ROS_INFO_STREAM("Euler angles: " << euler_angles);
  }
  double angle = euler_angles[0];

  // Add the new angle to samples
  // TODO: check if the angle is an outlier
  // Normalize angles close to ±180° to be close to 0°
  if (std::abs(angle) > M_PI / 2) {
    angle = (angle > 0) ? M_PI - angle : -M_PI - angle;
  }
  angle_samples_.push_back(std::abs(angle));
  if (angle_samples_.size() > MAX_SAMPLES) {
    angle_samples_.erase(angle_samples_.begin());
  }

  // Calculate statistics
  calculateStats();

  int front = object_centroid[0] < wall_centroid[0];
  std::string state_str;
  //! FIXME: make sure the angle is in the range of [0, pi/2] and correct
  if (angle < 0) {
    // front & left axis or back & right axis
    state_str = front ? "front & left" : "back & right";
  } else {
    // front & right axis or back & left axis
    state_str = front ? "front & right" : "back & left";
  }

  object_angle_.header.frame_id = base_frame_;
  object_angle_.header.stamp = ros::Time::now();
  object_angle_.ns = "object_angle";
  object_angle_.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  object_angle_.action = visualization_msgs::Marker::ADD;
  object_angle_.pose.position.x = object_centroid[0];
  object_angle_.pose.position.y = object_centroid[1];
  object_angle_.pose.position.z = object_centroid[2];
  object_angle_.color.r = 1.0f;
  object_angle_.color.g = 1.0f;
  object_angle_.color.b = 1.0f;
  object_angle_.color.a = 1.0;
  std::stringstream ss;
  ss << std::fixed << std::setprecision(2) << (abs(angle) * 180 / M_PI) << "°, "
     << state_str << "\nMean: " << (mean_angle_ * 180 / M_PI) << "°"
     << "\nStd Dev: " << (std_dev_ * 180 / M_PI) << "°"
     << "\nError Margin: ±" << (error_margin_ * 180 / M_PI) << "°";
  object_angle_.text = ss.str();
  object_angle_.scale.z = 0.5;
  object_angle_pub_.publish(object_angle_);
  if (enable_metrics_) {
    ROS_INFO_STREAM("Object angle stats: " << ss.str());
  }
  return;
}

void wall_detection::calculateStats() {
  if (angle_samples_.empty()) {
    mean_angle_ = 0.0;
    std_dev_ = 0.0;
    error_margin_ = 0.0;
    return;
  }

  // Calculate mean
  mean_angle_ =
      std::accumulate(angle_samples_.begin(), angle_samples_.end(), 0.0) /
      angle_samples_.size();

  // Calculate standard deviation
  double sq_sum = std::inner_product(
      angle_samples_.begin(), angle_samples_.end(), angle_samples_.begin(), 0.0,
      std::plus<double>(), [this](double a, double b) {
        return (a - mean_angle_) * (b - mean_angle_);
      });

  // Use n-1 for sample standard deviation (Bessel's correction)
  std_dev_ = std::sqrt(sq_sum / (angle_samples_.size() - 1));

  // Calculate error margin using Central Limit Theorem
  // Error Margin = Z * (σ / √n)
  error_margin_ = Z_SCORE_95 * (std_dev_ / std::sqrt(angle_samples_.size()));
}