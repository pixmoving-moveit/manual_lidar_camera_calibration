#! /usr/bin/python3

import numpy as np
import cv2
from scipy.spatial.transform import Rotation as R

import threading
import rclpy
import rclpy.node
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import CameraInfo
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster




class ExtrinsicCalibrator(rclpy.node.Node):
  def __init__(self):
    super().__init__('ExtrinsicCalibrator')
    
    self.declare_parameter('camera_frame_id', 'camera')
    self.camera_frame_id = self.get_parameter('camera_frame_id').get_parameter_value().string_value
    self.declare_parameter('lidar_frame_id', 'lidar')
    self.lidar_frame_id = self.get_parameter('lidar_frame_id').get_parameter_value().string_value
    self.declare_parameter('camera_info_topic', '/camera_info')
    self.camera_info_topic = self.get_parameter('camera_info_topic').get_parameter_value().string_value
    
    self.K = np.eye(3)
    
    self.camera_points_subscriber = self.create_subscription(
      Float32MultiArray,
      'camera_points',
      self.cameraPointsCallback,
      10
    )
    self.object_poins_subscriber = self.create_subscription(
      PointStamped,
      '/clicked_point',
      self.objectPointsCallback,
      10
    )
    self.camera_info_subscriber = self.create_subscription(
      CameraInfo,
      self.camera_info_topic,
      self.cameraInfoCallback, 10
    )
    
    self.camera_points = []
    self.object_points = []

    self.is_camera_info_received = False
    
    # ROS timer
    self.timer = self.create_timer(0.5, self.timerCallback)
    
    self.num_points = 0
    
    self.tf_static_broadcaster = StaticTransformBroadcaster(self)
  
  def make_transforms(self, quat, t_vec):
    t = TransformStamped()
    t.header.stamp = self.get_clock().now().to_msg()
    t.header.frame_id = self.camera_frame_id
    t.child_frame_id = self.lidar_frame_id
    t.transform.translation.x = float(t_vec[0][0])
    t.transform.translation.y = float(t_vec[1][0])
    t.transform.translation.z = float(t_vec[2][0])
    t.transform.rotation.x = quat[0]
    t.transform.rotation.y = quat[1]
    t.transform.rotation.z = quat[2]
    t.transform.rotation.w = quat[3]
    self.get_logger().info("sending new TF")
    self.tf_static_broadcaster.sendTransform(t)
  
  def extrinsicsCompute(self, object_points_list: list, image_points_list: list, k_matrix: list):
    object_points = np.array(object_points_list).reshape(len(object_points_list), 3, 1)
    image_points = np.array(image_points_list).reshape(len(object_points_list), 2, 1)
    camera_matrix = np.array(k_matrix).reshape(3, 3)
    dist_coeffs = np.zeros((5,1))
    ret_val, r_vec, t_vec = cv2.solvePnP(object_points, image_points, camera_matrix, dist_coeffs)
    if(ret_val):
      r_solved, _ = cv2.Rodrigues(r_vec)
      t_vec = t_vec.reshape(3, 1)
      homo_vec = np.array([[0.0, 0.0, 0.0, 1.0]])
      tf_matrix = np.hstack((r_solved, t_vec))
      tf_matrix = np.vstack((tf_matrix, homo_vec))

      # camera to lidar
      self.get_logger().info("transformation matrix:")
      self.get_logger().info(f"[[{tf_matrix[0][0]}, {tf_matrix[0][1]}, {tf_matrix[0][2]}, {tf_matrix[0][3]}], \n[{tf_matrix[1][0]}, {tf_matrix[1][1]}, {tf_matrix[1][2]}, {tf_matrix[1][3]}], \n[{tf_matrix[2][0]}, {tf_matrix[2][1]}, {tf_matrix[2][2]}, {tf_matrix[2][3]}], \n[{tf_matrix[3][0]}, {tf_matrix[3][1]}, {tf_matrix[3][2]}, {tf_matrix[3][3]}]\n]")
      r = R.from_matrix(r_solved)
      quat = r.as_quat()
      self.get_logger().info("quaternion: "+"[x, y, z, w]")
      self.get_logger().info(f"[{quat[0]}, {quat[1]}, {quat[2]}, {quat[3]}]")
      euler = r.as_euler('xyz')
      self.get_logger().info("euler angle: ")
      self.get_logger().info(f"x: {t_vec[0][0]}\ny: {t_vec[1][0]}\nz: {t_vec[2][0]}\nroll: {euler[0]}\npitch: {euler[1]}\nyaw: {euler[2]}")

      # lidar to camera
      tf_matrix_i = np.matrix(tf_matrix).I
      self.get_logger().info("invert transformation matrix:")
      self.get_logger().info(f"[[{tf_matrix_i[0, 0]}, {tf_matrix_i[0, 1]}, {tf_matrix_i[0, 2]}, {tf_matrix_i[0, 3]}], \n[{tf_matrix_i[1, 0]}, {tf_matrix_i[1, 1]}, {tf_matrix_i[1, 2]}, {tf_matrix_i[1, 3]}], \n[{tf_matrix_i[2, 0]}, {tf_matrix_i[2, 1]}, {tf_matrix_i[2, 2]}, {tf_matrix_i[2, 3]}], \n[{tf_matrix_i[3, 0]}, {tf_matrix_i[3, 1]}, {tf_matrix_i[3, 2]}, {tf_matrix_i[3, 3]}]\n]")
      r_i = R.from_matrix(tf_matrix_i[:3, :3])
      quat_i = r_i.as_quat()
      self.get_logger().info("quaternion invert: "+"[x, y, z, w]")
      self.get_logger().info(f"[{quat_i[0]}, {quat_i[1]}, {quat_i[2]}, {quat_i[3]}]")
      euler_i = r_i.as_euler('xyz')
      self.get_logger().info("euler angle invert: ")
      self.get_logger().info(f"x: {tf_matrix_i[0, 3]}\ny: {tf_matrix_i[1, 3]}\nz: {tf_matrix_i[2, 3]}\nroll: {euler_i[0]}\npitch: {euler_i[1]}\nyaw: {euler_i[2]}")
      return quat, t_vec
  
  def cameraInfoCallback(self, msg: CameraInfo):
    if not self.is_camera_info_received:
      for i in range(9):
        self.K[int(i/3), i%3] = msg.k[i]
      self.is_camera_info_received = True

  def cameraPointsCallback(self, msg: Float32MultiArray):
    if(len(msg.data)<2 and len(msg.data)/2==len(self.camera_points)):
      return
    point = []
    point.append(msg.data[-2])
    point.append(msg.data[-1])
    self.camera_points.append(point)
    self.get_logger().info("number of camera points: "+ str(len(self.camera_points)) + "  x: "+str(point[0]) + "  y: "+str(point[1]))
  
  def objectPointsCallback(self, msg: PointStamped):
    point = [msg.point.x, msg.point.y, msg.point.z]
    self.object_points.append(point)
    self.get_logger().info("number of object points: "+ str(len(self.object_points)) + "  x: "+str(point[0]) + "  y: "+str(point[1]) + "  z: "+str(point[2]))
    
  def timerCallback(self,):
    if self.is_camera_info_received:
      num_camera_points = len(self.camera_points)
      num_object_points = len(self.object_points)
      if(num_camera_points==num_object_points):
        if(num_camera_points > self.num_points):
          self.num_points += 1
          if(num_camera_points>=6):
            self.get_logger().info("calibrating...")
            quat, t_vec = self.extrinsicsCompute(self.object_points, self.camera_points, self.K)
            self.get_logger().info("calibrated")
            self.make_transforms(quat, t_vec)
    else:
      self.get_logger().warn("camera info not received.")

def main():
  rclpy.init()
  node = ExtrinsicCalibrator()
  rclpy.spin(node)

if __name__ == '__main__':
  main()