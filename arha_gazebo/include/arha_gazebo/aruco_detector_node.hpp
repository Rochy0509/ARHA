#ifndef ARHA_GAZEBO__ARUCO_DETECTOR_NODE_HPP_
#define ARHA_GAZEBO__ARUCO_DETECTOR_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <opencv2/aruco.hpp>

#include <memory>
#include <string>
#include <vector>

namespace arha_gazebo
{

/// Camera calibration model
struct CameraCalibration
{
  bool ready{false};
  cv::Mat camera_matrix;
  cv::Mat dist_coeffs;
  std::string optical_frame;
};

/// Detected ArUco marker
struct DetectedMarker
{
  int id;
  geometry_msgs::msg::PoseStamped pose;
  double confidence;
};

/**
 * @brief Simple ArUco marker detection node using chest camera.
 *
 * This node detects ArUco markers using the chest camera and publishes
 * their poses. It can be used for visual servoing, object localization,
 * and collision avoidance with point clouds.
 */
class ArucoDetectorNode : public rclcpp::Node
{
public:
  ArucoDetectorNode();

private:
  // ============================================================================
  // Callbacks
  // ============================================================================

  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  // ============================================================================
  // Helper Methods
  // ============================================================================

  void detectMarkers(const sensor_msgs::msg::Image::SharedPtr & msg);
  void publishMarkers(const std::vector<DetectedMarker> & markers);

  // ============================================================================
  // Member Variables
  // ============================================================================

  // TF2 for coordinate transformations
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // Camera calibration
  CameraCalibration camera_;

  // Subscriptions
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;

  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr marker_poses_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_cloud_pub_;

  // ArUco detection
  cv::Ptr<cv::aruco::Dictionary> dictionary_;
  cv::Ptr<cv::aruco::DetectorParameters> detector_params_;

  // Parameters
  double marker_size_;
  std::string camera_topic_;
  std::string target_frame_;
  bool publish_debug_images_;
};

}  // namespace arha_gazebo

#endif  // ARHA_GAZEBO__ARUCO_DETECTOR_NODE_HPP_
