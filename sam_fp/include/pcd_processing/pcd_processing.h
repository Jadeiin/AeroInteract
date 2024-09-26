#ifndef PCD_PROCESSING_CLASS_H
#define PCD_PROCESSING_CLASS_H

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <masks_msgs/maskID.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Char.h>
#include <tf/transform_listener.h>

#include <algorithm>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
// PCL specific includes
#include <pcl/ModelCoefficients.h>
#include <pcl/common/common.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <pcl_ros/impl/transforms.hpp>

/**
 * @brief: Class pcd_processing: cut RGB-D point cloud using 2D-masks generated
 * by Segment Anything from Meta.
 *
 *
 */
class pcd_processing {
 private:
  const std::string pointcloud_topic;
  const std::string base_frame;
  bool is_cloud_updated;  //!< new pointcloud recieved

 public:
  // Alias:
  typedef pcl::PointXYZRGB point;  // Point Type (vector type)
  typedef pcl::PointCloud<pcl::PointXYZRGB>
      cloud;  // PointCloud Type (cloud vector type)
  typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr
      cloudPtr;  // Cloud Pointer Type

  // Constructor and Destructor
  pcd_processing(const std::string &topic = "/camera/depth_registered/points",
                 const std::string &frame = "camera_link")
      : pointcloud_topic(topic),
        base_frame(frame),
        is_cloud_updated(false) {

  }  // Initialize and refer to topic and frame with default values. Initialize
     // member variables, allocate resources, etc.

  ~pcd_processing() {
    // Empty destructor body
  }  // Destructor

  /**
   * @brief initialize ros all subscribers/publishers, member variables
   *
   * @param nh NodeHandle
   * @return true success
   * @return false failure
   */
  bool initialize(ros::NodeHandle &nh);

  /**
   * @brief called periodically, update the pcd_processing object
   *
   * @param time
   */
  void update(const ros::Time &time);

 private:
  struct singlemask {
    Eigen::Matrix<int64_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>
        segmentation;
    int area;
    std::vector<int32_t> bbox;
    double predicted_iou;
    Eigen::Matrix<float, Eigen::Dynamic, 2, Eigen::RowMajor> point_coords;
    double stability_score;
    std::vector<int16_t> crop_box;
  };
  /**
   * @brief preprocessing the incoming raw point cloud, subsample and filter it.
   *
   * @param input raw point cloud data
   * @param output filterd and subsampled cloud data
   * @return true success
   * @return false failure
   */
  bool raw_cloud_preprocessing(cloudPtr &input, cloudPtr &output);

  /**
   * @brief cut the point cloud using masks generating from SAM
   *
   * @param input preprocessed point cloud
   * @param masks masks from SAM
   * @param objects objects that cut from point cloud
   * @return true success
   * @return false failure
   */
  bool cut_point_cloud(cloudPtr &input, const std::vector<singlemask> &masks,
                       cloudPtr &objects);

  /**
   * @brief extract bboxes from the point cloud
   *
   * @param input objects that cut from point cloud
   * @return true success
   * @return false failure
   */
  bool extract_bboxes(cloudPtr &input);

  /**
   * @brief callback function for new pointcloud subscriber
   *
   * @param msg
   */
  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg);

  /**
   * @brief callback function for new masks subscriber
   *
   * @param msg
   */
  void masksCallback(const masks_msgs::maskID::Ptr &msg);

  int countOnes(const Eigen::Matrix<int64_t, Eigen::Dynamic, Eigen::Dynamic,
                                    Eigen::RowMajor> &matrix);

  // Private variables

  ros::Subscriber point_cloud_sub_;   //!< Subscriber to the PointCloud data
  ros::Publisher objects_cloud_pub_;  //!< Publish objects point cloud
  ros::Subscriber masks_sub_;         //!< Subscriber to the masks data
  ros::Publisher object_boxes_pub_;   //!< Publish object bounding boxes
  cloudPtr raw_cloud_;                //!< Internal raw point cloud
  cloudPtr preprocessed_cloud_;       //!< Internal preprocessed cloud
  cloudPtr objects_cloud_;            //!< Internal objects point cloud
  masks_msgs::maskID::Ptr
      latest_maskID_msg_;              //!< Internal latest maskID message
  sensor_msgs::PointCloud2 cloudmsg_;  //!< save msg to cloudmsg_
  visualization_msgs::MarkerArray object_boxes_;  //!< object bounding box msg

  std::vector<singlemask> processed_masks_;  //!< Internal processed masks

  std::vector<singlemask> maskID_msg_processing(
      const masks_msgs::maskID::Ptr &maskID);

  // Transformation
  tf::TransformListener
      tf_listener_;  //!< Access ros tf tree to get frame transformations
};

#endif