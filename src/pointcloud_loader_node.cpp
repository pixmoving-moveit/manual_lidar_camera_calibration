#include <manual_lidar_camera_calibration/pointcloud_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <memory>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<manual_lidar_camera_calibration::pointcloud_loader::PointcloudLoader>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}