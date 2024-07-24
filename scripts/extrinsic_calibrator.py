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
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


def extrinsicsCompute(object_points_list: list, image_points_list: list, k_matrix: list):
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
    print("transformation matrix:")
    print(tf_matrix)
    r = R.from_matrix(r_solved)
    quat = r.as_quat()
    print("quaternion: "+"[x, y, z, w]")
    print(quat)
    euler = r.as_euler('zyx')
    print(f"x: {t_vec[0][0]}\ny: {t_vec[1][0]}\nz: {t_vec[2][0]}\nroll: {euler[2]}\npitch: {euler[1]}\nyaw: {euler[0]}")
    return quat, t_vec

class ExtrinsicCalibrator(rclpy.node.Node):
  def __init__(self):
    super().__init__('ExtrinsicCalibrator')

    self.K = np.array(
      [
        [1.0096461088041725e+03, 0., 9.3689248385778296e+02],
        [0., 1.0096581504805509e+03, 4.9988257013426869e+02],
        [0., 0., 1. ]
      ]
    ).reshape(3, 3)
    
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
    self.camera_points = []
    self.object_points = []
    
    # ROS timer
    self.timer = self.create_timer(0.5, self.timerCallback)
    
    self.num_points = 0
    
    self.tf_static_broadcaster = StaticTransformBroadcaster(self)
  
  def make_transforms(self, quat, t_vec):
    t = TransformStamped()
    t.header.stamp = self.get_clock().now().to_msg()
    t.header.frame_id = 'camera'
    t.child_frame_id = 'lidar'
    t.transform.translation.x = float(t_vec[0][0])
    t.transform.translation.y = float(t_vec[1][0])
    t.transform.translation.z = float(t_vec[2][0])
    t.transform.rotation.x = quat[0]
    t.transform.rotation.y = quat[1]
    t.transform.rotation.z = quat[2]
    t.transform.rotation.w = quat[3]
    print("sending new TF")
    self.tf_static_broadcaster.sendTransform(t)
  
  def cameraPointsCallback(self, msg: Float32MultiArray):
    if(len(msg.data)<2 and len(msg.data)/2==len(self.camera_points)):
      return
    point = []
    point.append(msg.data[-2])
    point.append(msg.data[-1])
    self.camera_points.append(point)
    print("number of camera points: "+ str(len(self.camera_points)) + "  x: "+str(point[0]) + "  y: "+str(point[1]))
  
  def objectPointsCallback(self, msg: PointStamped):
    point = [msg.point.x, msg.point.y, msg.point.z]
    self.object_points.append(point)
    print("number of object points: "+ str(len(self.object_points)) + "  x: "+str(point[0]) + "  y: "+str(point[1]) + "  z: "+str(point[2]))
    
  def timerCallback(self,):
    num_camera_points = len(self.camera_points)
    num_object_points = len(self.object_points)
    if(num_camera_points==num_object_points):
      if(num_camera_points > self.num_points):
        self.num_points += 1
        if(num_camera_points>=6):
          print("calibrating...")
          quat, t_vec = extrinsicsCompute(self.object_points, self.camera_points, self.K)
          print("calibrated")
          self.make_transforms(quat, t_vec)

def main():
  rclpy.init()
  node = ExtrinsicCalibrator()
  rclpy.spin(node)

if __name__ == '__main__':
  main()