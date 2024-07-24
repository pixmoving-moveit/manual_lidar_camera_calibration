#include "manual_lidar_camera_calibration/pointcloud_loader.hpp"

using namespace std::chrono_literals;

namespace manual_lidar_camera_calibration
{
namespace pointcloud_loader
{
PointcloudLoader::PointcloudLoader() : Node("manual_lidar_camera_calibration_pointcloud_loader")
{
  pcd_path_ = declare_parameter("pcd_path", "pointcloud.pcd");
  leaf_size_ = declare_parameter("leaf_size", 0.01);

  pointcloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "pointcloud", rclcpp::QoS(1).transient_local());
  timer_ = this->create_wall_timer(
    500ms, std::bind(&PointcloudLoader::timerCallback, this));
  
  // approximate_voxel_filter_.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
  // pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  // approximate_voxel_filter_.setInputCloud(raw_cloud);
  // approximate_voxel_filter_.filter(*filtered_cloud);
  
  
}

void PointcloudLoader::timerCallback()
{
  if(!is_published_)
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr raw_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::io::loadPCDFile<pcl::PointXYZI>(pcd_path_, *raw_cloud);
    pcl::toROSMsg(*raw_cloud, points_msg_);
    points_msg_.header.stamp = this->now();
    points_msg_.header.frame_id = "lidar";
    pointcloud_publisher_->publish(points_msg_);
  }
  is_published_ = true;
}

PointcloudLoader::~PointcloudLoader()
{
}
}
}