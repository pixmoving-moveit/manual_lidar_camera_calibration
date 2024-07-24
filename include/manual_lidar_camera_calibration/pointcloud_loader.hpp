#ifndef __MANUAL_LIDAR_CAMERA_CALIBRATION__POINTCLOUD_LOADER_HPP__
#define __MANUAL_LIDAR_CAMERA_CALIBRATION__POINTCLOUD_LOADER_HPP__

#include <string>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace manual_lidar_camera_calibration
{
namespace pointcloud_loader
{

class PointcloudLoader : public rclcpp::Node
{
  private:
    // data publisher
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_publisher_;
    // file path
    std::string pcd_path_;
    // voxel grid downsample leaf size
    double leaf_size_;

    bool is_published_=false;

    pcl::ApproximateVoxelGrid<pcl::PointXYZI> approximate_voxel_filter_;

    sensor_msgs::msg::PointCloud2 points_msg_;
    rclcpp::TimerBase::SharedPtr timer_;
  public:
    PointcloudLoader();
    ~PointcloudLoader();
    void timerCallback();
};
}
}


#endif // __MANUAL_LIDAR_CAMERA_CALIBRATION__DATA_LOADER_HPP__