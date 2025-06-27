#! /usr/bin/python3

import threading
import cv2
import numpy as np
import rclpy
import rclpy.node
from cv_bridge import CvBridge
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo


class CameraPointsPublisher(rclpy.node.Node):
  def __init__(self):
    super().__init__('camera_points_publisher')
    self.declare_parameter('image_topic', '/image_raw')
    self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
    self.declare_parameter('camera_info_topic', '/camera_info')
    self.camera_info_topic = self.get_parameter('camera_info_topic').get_parameter_value().string_value
    self.declare_parameter('output_camera_frame_id', 'camera')
    self.output_camera_frame_id = self.get_parameter('output_camera_frame_id').get_parameter_value().string_value

    self.img = None
    self.K = np.eye(3)
    self.D = np.zeros((14, 1))

    self.ud_frame = None

    # ROS msgs
    # image msg
    self.image_msg = None
    
    # camera info msg
    self.camera_info_msg = None
    
    # camera points msg
    self.camera_points_msg = Float32MultiArray()
    # ROS publisher
    self.image_publisher = self.create_publisher(Image, "image_raw", 10)
    self.camera_info_publisher = self.create_publisher(CameraInfo, "camera_info", 10)
    self.camera_points_publisher = self.create_publisher(Float32MultiArray, "camera_points", 10)
    # ROS subscriber
    self.image_subscriber = self.create_subscription(Image, self.image_topic, self.imageCallback, 10)
    self.camera_info_subscriber = self.create_subscription(CameraInfo, self.camera_info_topic, self.cameraInfoCallback, 10)
    # ROS timer
    self.timer = self.create_timer(1.0, self.timerCallback)

    self.is_image_received = False
    self.is_camera_info_received = False
    self.is_showing_image = False

    self.camera_points = []
    
  def timerCallback(self,):
    if self.is_image_received and self.is_camera_info_received:
      if not self.is_showing_image:
        self.ud_frame = self.distortionCorrection(self.K, self.D, self.img)
        threading.Thread(target=self.showImage).start()
        self.is_showing_image = True
    else:
      self.get_logger().warn("image or camera info not received.")
  
  def imageCallback(self, msg: Image):
    if not self.is_image_received:
      self.img = CvBridge().imgmsg_to_cv2(msg)
      self.is_image_received = True
    self.image_msg = msg
    self.image_msg.header.frame_id = self.output_camera_frame_id
    self.image_publisher.publish(self.image_msg)
  
  def cameraInfoCallback(self, msg: CameraInfo):
    if not self.is_camera_info_received:
      for i in range(9):
        self.K[int(i/3), i%3] = msg.k[i]
      for i in range(len(msg.d)):
        self.D[i, 0] = msg.d[i]
      self.is_camera_info_received = True
    self.camera_info_msg = msg
    self.camera_info_msg.header.frame_id = self.output_camera_frame_id
    self.camera_info_publisher.publish(self.camera_info_msg)
  
  def undistort(self, k_matrix: np.array, d_matrix: np.array, frame):
    h, w = frame.shape[:2]
    mapx, mapy = cv2.initUndistortRectifyMap(k_matrix, d_matrix, None, k_matrix, (w, h), 5)
    return cv2.remap(frame, mapx, mapy, cv2.INTER_LINEAR)

  def distortionCorrection(self, k_matrix: np.array, d_matrix: np.array, frame):
    undistort_frame = self.undistort(k_matrix, d_matrix, frame)
    return undistort_frame

  def EVENT_LBUTTONDOWN(self, event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
      self.camera_points_msg.data = []
      self.camera_points_msg.data.append(float(x))
      self.camera_points_msg.data.append(float(y))
      self.camera_points_publisher.publish(self.camera_points_msg)
      xy_coor = "%d, %d" %(x, y)
      print(xy_coor)
      cv2.circle(self.ud_frame, (x, y), 1, (133, 21, 199), thickness=5)
      cv2.putText(self.ud_frame, xy_coor, (x, y), cv2.FONT_HERSHEY_PLAIN, 2.0, (255, 255, 255), thickness=2)
      cv2.imshow("frame", self.ud_frame)
  
  def showImage(self, ):
    cv2.namedWindow("frame", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("frame", 1600, 900)
    cv2.imshow('frame', self.ud_frame)
    cv2.setMouseCallback("frame", self.EVENT_LBUTTONDOWN)
    cv2.waitKey(0)
    cv2.destroyAllWindows() 

def main():
  rclpy.init()
  node = CameraPointsPublisher()
  rclpy.spin(node)

if __name__ == '__main__':
  main()