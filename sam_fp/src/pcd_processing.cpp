#include "pcd_processing/pcd_processing.h"

// Constructor
pcd_processing::pcd_processing(ros::NodeHandle& nh_) : nh(nh_) {
  // Get parameters from the parameter server
  nh.param<std::string>("pointcloud_topic", pointcloud_topic_, "/camera/depth_registered/points");
  nh.param<std::string>("base_frame", base_frame_, "camera_link");
  nh.param<bool>("/enable_metrics", enable_metrics_, false);

  // Initialize ROS subscribers, publishers, and other members
  point_cloud_sub_ =
      nh.subscribe(pointcloud_topic_, 10, &pcd_processing::cloudCallback, this);
  masks_sub_ =
      nh.subscribe("/sam_mask", 10, &pcd_processing::masksCallback, this);
  objects_cloud_pub_ =
      nh.advertise<sensor_msgs::PointCloud2>("/objects_cloud", 10);
  background_cloud_pub_ =
      nh.advertise<sensor_msgs::PointCloud2>("/background_cloud", 10);
  objects_marker_pub_ =
      nh.advertise<visualization_msgs::MarkerArray>("/objects_marker", 10);

  // Initialize pointers
  raw_cloud_.reset(new cloud);
  preprocessed_cloud_.reset(new cloud);
  objects_cloud_.reset(new cloud);
  background_cloud_.reset(new cloud);
  latest_maskID_msg_.reset(new masks_msgs::maskID);

  // Initialize flags
  is_cloud_updated = false;
}

// Update method
void pcd_processing::update(const ros::Time &time) {
  // Update the pcd_processing object

  if (is_cloud_updated) {
    // Preprocess the raw cloud
    ros::Time start;
    if (enable_metrics_) start = ros::Time::now();
    if (!raw_cloud_preprocessing(raw_cloud_, preprocessed_cloud_)) {
      ROS_ERROR("Raw cloud preprocessing failed!");
      return;
    }
    if (enable_metrics_) {
      ros::Time end = ros::Time::now();
      ROS_INFO_STREAM("Raw cloud preprocessing time: " << (end - start).toNSec()
                                                     << " ns");
    }

    // Cut the preprocessed cloud //TODO: pass the argument
    if (enable_metrics_) start = ros::Time::now();
    if (!cut_point_cloud(preprocessed_cloud_, processed_masks_, objects_cloud_,
                         background_cloud_)) {
      ROS_ERROR("Cutting point cloud failed!");
      return;
    }
    if (enable_metrics_) {
      ros::Time end = ros::Time::now();
      ROS_INFO_STREAM("Cutting point cloud time: " << (end - start).toNSec()
                                                   << " ns");
    }

    if (enable_metrics_) start = ros::Time::now();
    if (!segment_plane(objects_cloud_)) {
      ROS_ERROR("Segmenting plane failed!");
      return;
    }
    if (enable_metrics_) {
      ros::Time end = ros::Time::now();
      ROS_INFO_STREAM("Segmenting plane time: " << (end - start).toNSec()
                                               << " ns");
    }

    // Publish the objects cloud
    pcl::toROSMsg(*objects_cloud_, cloudmsg_);
    // ROS_INFO_STREAM("raw_cloud_:");
    // ROS_INFO_STREAM(*raw_cloud_);
    // ROS_INFO_STREAM("objects_cloud_:");
    // ROS_INFO_STREAM(*objects_cloud_);
    objects_cloud_pub_.publish(cloudmsg_);
    pcl::toROSMsg(*background_cloud_, cloudmsg_);
    background_cloud_pub_.publish(cloudmsg_);
    // ROS_INFO_STREAM("objects_marker_:");
    // ROS_INFO_STREAM(objects_marker_);
    objects_marker_pub_.publish(objects_marker_);

    // Reset the flag
    is_cloud_updated = false;
  }
}

// Raw cloud preprocessing
bool pcd_processing::raw_cloud_preprocessing(cloudPtr &input,
                                             cloudPtr &output) {
  // Modify if further preprocessing needed
  // Note: Since the point cloud segmentation is pixel-wise, preprocessing may
  // cause lack of points.

  *output = *input;

  return true;  // Return true on success
}

// Cut point cloud
bool pcd_processing::cut_point_cloud(cloudPtr &input,
                                     const std::vector<singlemask> &masks,
                                     cloudPtr &objects, cloudPtr &background) {
  // Implement the logic to cut the point cloud using masks
  // Point Cloud frame_id: camera_color_optical_frame
  // image_raw frame_id: camera_color_optical_frame
  // masks frame_id: camera_color_optical_frame

  // Clear the output cloud
  *objects = *input;
  objects->points.clear();

  // Iterate over each mask
  // ROS_INFO_STREAM("masks:");
  // ROS_INFO_STREAM(masks.size());
  for (const auto &mask : masks) {
    // Find the bounding box of the mask
    int min_x = mask.bbox[0];
    int min_y = mask.bbox[1];
    int max_x = mask.bbox[2];
    int max_y = mask.bbox[3];

    // int number_of_ones = pcd_processing::countOnes(mask.segmentation);
    // ROS_INFO_STREAM("number_of_ones:");
    // ROS_INFO_STREAM(number_of_ones);

    pcl::ExtractIndices<point> extract;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    int cols = mask.segmentation.cols();
    // Iterate over the points in the bounding box
    for (int i = min_y; i < max_y; ++i) {
      for (int j = min_x; j < max_x; ++j) {
        // Check if the mask includes this point
        if (mask.segmentation(i, j) == 1) {
          // Calculate the index in the point cloud
          int index = i * cols + j;
          if (index < input->points.size()) {
            // Add the point to the inliers list
            inliers->indices.push_back(index);
          }
        }
      }
    }
    // Extract the points from the object cloud
    extract.setInputCloud(input);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*objects);
    // Extract the points from the background cloud
    extract.setNegative(true);
    extract.filter(*background);
  }
  objects->width = objects->points.size();
  objects->height = 1;  // Setting height to 1 implies the cloud is unorganized
  objects->is_dense =
      false;  // Set to false if there might be NaN or invalid points

  // Remove NaN points from the processed cloud
  // std::vector<int> indices;
  // pcl::removeNaNFromPointCloud(*objects, *objects, indices);
  // pcl::removeNaNFromPointCloud(*background, *background, indices);

  // Downsample the point cloud
  pcl::VoxelGrid<point> voxel_grid;
  double voxel_size = 0.02;
  voxel_grid.setInputCloud(objects);
  voxel_grid.setLeafSize(voxel_size, voxel_size, voxel_size);
  voxel_grid.filter(*objects);
  voxel_grid.setInputCloud(background);
  voxel_grid.setLeafSize(voxel_size, voxel_size, voxel_size);
  voxel_grid.filter(*background);

  return true;
}

bool pcd_processing::segment_plane(cloudPtr &input) {
  // Implement the logic to segement plane from the point cloud

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
  seg.setInputCloud(input);
  seg.segment(*inliers, *coefficients);
  if (inliers->indices.size() == 0) {
    ROS_ERROR(
        "Could not estimate a planar model for the given dataset (size = 0).");
    return false;
  }
  if (inliers->indices.size() < 500) {
    ROS_ERROR(
        "Could not estimate a planar model for the given dataset (size = %ld).",
        inliers->indices.size());
    return false;
  }
  pcl::copyPointCloud(*input, inliers->indices, *input);
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(*input, centroid);

  geometry_msgs::PointStamped start_base, start_cam, end_base;
  geometry_msgs::Vector3Stamped normal_base, normal_cam;
  start_cam.header.frame_id = normal_cam.header.frame_id =
      input->header.frame_id;
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
    return false;
  }
  end_base.point.x = start_base.point.x + normal_base.vector.x;
  end_base.point.y = start_base.point.y + normal_base.vector.y;
  end_base.point.z = start_base.point.z + normal_base.vector.z;

  visualization_msgs::Marker arrow_marker;
  arrow_marker.header.frame_id = base_frame_;
  arrow_marker.header.stamp = ros::Time::now();
  arrow_marker.ns = "object_arrow";
  arrow_marker.type = visualization_msgs::Marker::ARROW;
  arrow_marker.action = visualization_msgs::Marker::ADD;
  arrow_marker.points.push_back(start_base.point);
  arrow_marker.points.push_back(end_base.point);
  arrow_marker.color.r = 0.0f;
  arrow_marker.color.g = 1.0f;
  arrow_marker.color.b = 0.0f;
  arrow_marker.color.a = 1.0;
  arrow_marker.scale.x = 0.1;
  arrow_marker.scale.y = 0.1;
  arrow_marker.scale.z = 0.1;
  objects_marker_.markers.clear();
  objects_marker_.markers.push_back(arrow_marker);

  return true;
}

// Cloud callback
void pcd_processing::cloudCallback(
    const sensor_msgs::PointCloud2ConstPtr &msg) {
  is_cloud_updated = true;

  pcl::fromROSMsg(*msg, *raw_cloud_);
}

// Masks callback
void pcd_processing::masksCallback(const masks_msgs::maskID::Ptr &msg) {
  // process new recieved masks
  processed_masks_ = maskID_msg_processing(msg);
}

std::vector<pcd_processing::singlemask> pcd_processing::maskID_msg_processing(
    const masks_msgs::maskID::Ptr &maskID) {
  ROS_INFO("mask_msg_preprocessing is triggered.");

  std::vector<singlemask> masks;
  for (const auto &singlemask_msg : maskID->maskID) {
    singlemask mask;
    mask.maskid = singlemask_msg.maskid;
    mask.segmentation =
        Eigen::Map<const Eigen::Matrix<int64_t, Eigen::Dynamic, Eigen::Dynamic,
                                       Eigen::RowMajor>>(
            singlemask_msg.segmentation.data(), singlemask_msg.shape[0],
            singlemask_msg.shape[1]);
    mask.area = singlemask_msg.area;
    mask.bbox = singlemask_msg.bbox;
    ROS_INFO_STREAM("mask info: id="
                    << mask.maskid << ",shape=(" << singlemask_msg.shape[0]
                    << "," << singlemask_msg.shape[1] << "),bbox=("
                    << mask.bbox[0] << "," << mask.bbox[1] << ","
                    << mask.bbox[2] << "," << mask.bbox[3] << ")");
    masks.push_back(mask);
  }

  ROS_INFO_STREAM("length of masks before erase:");
  ROS_INFO_STREAM(masks.size());

  // Sort the masks by area
  auto compareArea = [](const singlemask &a, const singlemask &b) {
    return a.area < b.area;
  };
  std::sort(masks.begin(), masks.end(), compareArea);
  // Erase the masks with the largest area (the background mask)
  if (masks.size() > 5) {
    masks.erase(masks.end() - 5, masks.end());
  }

  return masks;
}

int pcd_processing::countOnes(
    const Eigen::Matrix<int64_t, Eigen::Dynamic, Eigen::Dynamic,
                        Eigen::RowMajor> &matrix) {
  int count = 0;
  for (int i = 0; i < matrix.rows(); i++) {
    for (int j = 0; j < matrix.cols(); j++) {
      if (matrix(i, j) == 1) {
        count++;
      }
    }
  }
  return count;
}
