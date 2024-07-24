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
    self.declare_parameter('image_path', 'image.png')
    self.image_path = self.get_parameter('image_path').get_parameter_value().string_value
    self.img = cv2.imread(self.image_path, 1)
    self.K = np.array(
      [
        [1.0096461088041725e+03, 0., 9.3689248385778296e+02],
        [0., 1.0096581504805509e+03, 4.9988257013426869e+02],
        [0., 0., 1. ]
      ]
    ).reshape(3, 3)
    self.D = np.array(
      [
        9.5856164145301237e-01,
        1.8914835820355605e-01,
        -2.4506997072279488e-06,
        5.9923693345781568e-05,
        3.9186197392106761e-03,
        1.3529369410,
        0.4698847334,
        0.0333375880
      ]
    ).reshape(8, 1)
    # self.D = np.zeros((5, 1))

    self.ud_frame = self.distortionCorrection(self.K, self.D, self.img)
    # cv2.imwrite("image.png", self.ud_frame)
    threading.Thread(target=self.showImage).start()

    # ROS msgs
    # image msg
    self.image_msg = CvBridge().cv2_to_imgmsg(self.ud_frame)
    self.image_msg.header.frame_id = "camera"
    
    # camera info msg
    self.camera_info_msg = CameraInfo()
    self.camera_info_msg.height = 1080
    self.camera_info_msg.width = 1920
    self.camera_info_msg.distortion_model = "plumb_bob"
    self.camera_info_msg.header.frame_id = "camera"
    for i in range(self.D.size):
      self.camera_info_msg.d.append(self.D[i][0])
    for i in range(9):
      self.camera_info_msg.k[i] = self.K[int(i/3)][i%3]
    self.camera_info_msg.r[0] = 1.0
    self.camera_info_msg.r[4] = 1.0
    self.camera_info_msg.r[8] = 1.0
    self.camera_info_msg.p[0] = self.camera_info_msg.k[0]
    self.camera_info_msg.p[2] = self.camera_info_msg.k[2]
    self.camera_info_msg.p[5] = self.camera_info_msg.k[4]
    self.camera_info_msg.p[6] = self.camera_info_msg.k[5]
    self.camera_info_msg.p[10] = self.camera_info_msg.k[8]
    
    # camera points msg
    self.camera_points_msg = Float32MultiArray()
    # ROS publisher
    self.image_publisher = self.create_publisher(Image, "image_raw", 10)
    self.camera_info_publisher = self.create_publisher(CameraInfo, "camera_info", 10)
    self.camera_points_publisher = self.create_publisher(Float32MultiArray, "camera_points", 10)
    # ROS timer
    self.timer = self.create_timer(1.0, self.timerCallback)

    self.camera_points = []
    
  def timerCallback(self,):
    self.image_msg.header.stamp = self.get_clock().now().to_msg()
    self.camera_info_msg.header.stamp = self.get_clock().now().to_msg()
    self.image_publisher.publish(self.image_msg)
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