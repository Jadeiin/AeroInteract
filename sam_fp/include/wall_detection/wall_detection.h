#ifndef WALL_DETECTION_CLASS_H
#define WALL_DETECTION_CLASS_H

#include <pcl/ModelCoefficients.h>
#include <pcl/common/centroid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <numeric>
#include <vector>

/**
 * @brief The wall_detection class is responsible for detecting walls in point
 * cloud data and calculating the angle between detected walls and objects.
 */
class wall_detection {
 public:
  // Alias:
  typedef pcl::PointXYZRGB point;  // Point Type (vector type)
  typedef pcl::PointCloud<pcl::PointXYZRGB>
      cloud;  // PointCloud Type (cloud vector type)
  typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr
      cloudPtr;  // Cloud Pointer Type

  /**
   * @brief Constructor for wall_detection class
   *
   * @param nh_ ROS NodeHandle
   */
  wall_detection(ros::NodeHandle &nh_);
  ~wall_detection() {}

 private:
  /**
   * @brief Callback function for processing incoming point cloud data.
   * @param msg The incoming point cloud message.
   */
  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg);

  /**
   * @brief Callback function for processing incoming object marker data.
   * @param msg The incoming marker array message.
   */
  void objectCallback(const visualization_msgs::MarkerArrayConstPtr &msg);

  /**
   * @brief Calculate statistics (mean, standard deviation, error margin) for
   * the angle samples.
   */
  void calculateStats();

  ros::NodeHandle nh;                //!< ROS NodeHandle
  ros::Subscriber point_cloud_sub_;  //!< Subscriber to the PointCloud data
  ros::Subscriber
      objects_marker_sub_;           //!< Subscriber to the objects marker data
  ros::Publisher wall_points_pub_;   //!< Publisher for wall points cloud
  ros::Publisher wall_marker_pub_;   //!< Publisher for wall marker
  ros::Publisher object_angle_pub_;  //!< Publisher for object angle

  cloudPtr raw_cloud_;                 //!< Internal raw point cloud
  cloudPtr wall_cloud_;                //!< Internal wall point cloud
  sensor_msgs::PointCloud2 cloudmsg_;  //!< Message to store point cloud data
  visualization_msgs::Marker wall_marker_;   //!< Wall marker message
  visualization_msgs::Marker object_angle_;  //!< Object angle marker message
  std::string pointcloud_topic_;  //!< Topic name for point cloud data
  std::string base_frame_;        //!< Base frame for transformations
  bool enable_metrics_;           //!< Flag to enable or disable metrics logging

  // Transformation
  tf::TransformListener
      tf_listener_;  //!< Access ROS tf tree to get frame transformations

  // Statistics
  std::vector<double>
      angle_samples_;    //!< Samples of angles between walls and objects
  double mean_angle_;    //!< Mean angle of the samples
  double std_dev_;       //!< Standard deviation of the angle samples
  double error_margin_;  //!< Error margin for 95% confidence interval
  static constexpr int MAX_SAMPLES = 50;  //!< Store last 50 samples
  static constexpr double Z_SCORE_95 =
      1.96;  //!< Z-score for 95% confidence level
};

#endif