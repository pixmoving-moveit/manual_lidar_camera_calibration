import numpy as np
import cv2
from scipy.spatial.transform import Rotation as R


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
    print("transform matrix: \n", tf_matrix)
  
if __name__ == "__main__":
  A = [
    [9.722850799560547, 8.36107349395752, 0.7034699320793152],
    [9.196243286132812, 6.273510456085205, 1.0500380992889404],
    [9.274091720581055, 3.0916430950164795, 0.20940369367599487],
    [4.74496603012085, 0.29412102699279785, 0.5698382258415222],
    [3.2251763343811035, -2.183462381362915, -0.2879936099052429],
    [6.4439311027526855, -4.9807538986206055, 0.27658897638320923],
    [3.449563980102539, -1.6144179105758667, 0.6617656350135803],
    [21.731534957885742, 11.238630294799805, 3.891907215118408],
    [11.678573608398438, 8.770978927612305, 2.177833080291748],
    [4.739025115966797, -0.4895790219306946, 0.5902748107910156],
    [3.36117601394653, -1.3943852186203003, -0.29041385650634766],
    [9.4654541015625, 7.885610580444336, -0.10759514570236206]
  ]

  B = [
    [58.0, 524.0],
    [227.0, 489.0],
    [464.0, 488.0],
    [756.0, 496.0],
    [1588.0, 796.0],
    [1777.0, 603.0],
    [1317.0, 428.0],
    [459.0, 424.0],
    [202.0, 412.0],
    [945.0, 494.0],
    [1249.0, 777.0],
    [82.0, 604.0]
  ]

  C = [
    [1.0096461088041725e+03, 0., 9.3689248385778296e+02],
    [0., 1.0096581504805509e+03, 4.9988257013426869e+02],
    [0., 0., 1. ]
    ]
  extrinsicsCompute(A, B, C)