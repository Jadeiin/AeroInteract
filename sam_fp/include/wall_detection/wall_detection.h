#ifndef WALL_DETECTION_CLASS_H
#define WALL_DETECTION_CLASS_H

#include <pcl/common/centroid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_msgs/ModelCoefficients.h>
#include <pcl_ros/io/pcd_io.h>
#include <pcl_ros/segmentation/sac_segmentation.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/tf.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

class wall_detection {
 public:
  // Alias:
  typedef pcl::PointXYZRGB point;  // Point Type (vector type)
  typedef pcl::PointCloud<pcl::PointXYZRGB>
      cloud;  // PointCloud Type (cloud vector type)
  typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr
      cloudPtr;  // Cloud Pointer Type

  wall_detection(const std::string &topic = "/camera/depth_registered/points")
      : pointcloud_topic(topic) {
    point_cloud_sub_ =
        nh_.subscribe(pointcloud_topic, 10, &wall_detection::cloudCallback, this);
    object_boxes_sub_ =
        nh_.subscribe("/object_boxes", 10, &wall_detection::obbCallback, this);
    wall_points_pub_ =
        nh_.advertise<sensor_msgs::PointCloud2>("/wall_points", 10);
    wall_marker_pub_ =
        nh_.advertise<visualization_msgs::Marker>("/wall_marker", 10);
    object_angle_pub_ =
        nh_.advertise<visualization_msgs::Marker>("/object_angle", 10);
    raw_cloud_.reset(new cloud);
    wall_cloud_.reset(new cloud);
  }

  ~wall_detection() {}

 private:
  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg);
  void obbCallback(const visualization_msgs::MarkerArrayConstPtr &msg);
  ros::NodeHandle nh_;
  ros::Subscriber point_cloud_sub_;
  ros::Subscriber object_boxes_sub_;
  ros::Publisher wall_points_pub_;
  ros::Publisher wall_marker_pub_;
  ros::Publisher object_angle_pub_;

  cloudPtr raw_cloud_;
  cloudPtr wall_cloud_;
  sensor_msgs::PointCloud2 cloudmsg_;
  visualization_msgs::Marker wall_marker_;
  visualization_msgs::Marker object_angle_;
  const std::string pointcloud_topic;
};

#endif