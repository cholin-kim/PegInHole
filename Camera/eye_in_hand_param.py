import numpy as np

# header:
#   seq: 435
#   stamp:
#     secs: 1732793736
#     nsecs: 516251564
#   frame_id: "camera_color_optical_frame"
# height: 480
# width: 848
# distortion_model: "plumb_bob"
# D: [-0.05458524078130722, 0.057370759546756744, 0.00011702199117280543, 0.0012725829146802425, -0.018503589555621147]
# K: [430.583740234375, 0.0, 419.8208312988281, 0.0, 430.1481628417969, 239.1549835205078, 0.0, 0.0, 1.0]
# R: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0] <- Rectification matrix, not used when using mono cam.
# P: [430.583740234375, 0.0, 419.8208312988281, 0.0, 0.0, 430.1481628417969, 239.1549835205078, 0.0, 0.0, 0.0, 1.0, 0.0]
# binning_x: 0
# binning_y: 0
# roi:
#   x_offset: 0
#   y_offset: 0
#   height: 0
#   width: 0
#   do_rectify: False
# ---


# sensor_msgs/CameraInfo.msg
'''
The internal parameters can be used to warp a raw(=distorted) image to:
    1. An undistorted img(requires D and K)
    2. A rectified img(requires D, K, R)
'''
img_width = 848
img_height = 480


# Distortion Parameters
D = np.array([-0.05458524078130722, 0.057370759546756744, 0.00011702199117280543, 0.0012725829146802425, -0.018503589555621147])

# Intrinsic matrix
# [[f_x, 0, c_x],
# [0, f_y, c_y],
# [0, 0, 1]]
K = np.array([[430.583740234375, 0.0, 419.8208312988281],
              [0.0, 430.1481628417969, 239.1549835205078],
              [0.0, 0.0, 1.0]])

# Projection matrix
#     [fx'  0  cx' Tx]
# P = [ 0  fy' cy' Ty]
#     [ 0   0   1   0]
P = np.array([[430.583740234375, 0.0, 419.8208312988281, 0.0],
              [0.0, 430.1481628417969, 239.1549835205078, 0.0],
              [0.0, 0.0, 1.0, 0.0]])

Tee_cam = np.array([[-0.707564,  -0.706569,  0.0106639,  0.0699855],
                    [0.706628,  -0.707579, 0.00293428,  0.0594376],
                    [0.00547228,  0.0096116,   0.999939,  0.0843903],
                    [0,          0,         0,          1]])


# R.from_matrix(Tee_cam[:3,:3]).as_euler('XYZ', degrees=True)
# array([ -0.16813161,   0.61100795, 135.04031963]) -> calibration 잘 된 것 확인
